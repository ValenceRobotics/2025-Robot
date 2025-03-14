package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.RobotState;
import frc.robot.RobotState.ElevatorState;
import frc.robot.util.ElevatorMath;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim m_elevatorSim =
      new ElevatorSim(
          ElevatorConstants.m_elevatorMotors,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMassKg,
          ElevatorConstants.kElevatorDrumRadiusMeters,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0);

  private SparkMax max = new SparkMax(0, MotorType.kBrushless);
  private SparkMaxSim maxSim;
  private SparkClosedLoopController elevatorController;

  private LoggedTunableNumber l4Position = new LoggedTunableNumber("Elevator/L4Position", 31.8);
  private LoggedTunableNumber l4ScorePosition =
      new LoggedTunableNumber("Elevator/L4Position", 32.5);
  private LoggedTunableNumber l1Position = new LoggedTunableNumber("Elevator/L1Position", 5.1);
  private LoggedTunableNumber l2Position = new LoggedTunableNumber("Elevator/L2Position", 9.3);
  private LoggedTunableNumber l3Position = new LoggedTunableNumber("Elevator/L3Position", 18.3);

  public ElevatorIOSim() {

    elevatorController = max.getClosedLoopController();
    var elevatorConfig = new SparkMaxConfig();

    elevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12.0);
    elevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kPSim, 0, ElevatorConstants.kDSim)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(ElevatorConstants.maxVelocity)
        .maxAcceleration(ElevatorConstants.maxAcceleration)
        .allowedClosedLoopError(ElevatorConstants.allowedError);
    Logger.recordOutput("Elevator/MaxVelocity", ElevatorConstants.maxVelocity);
    Logger.recordOutput("Elevator/MaxAcceleration", ElevatorConstants.maxAcceleration);
    max.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    maxSim = new SparkMaxSim(max, ElevatorConstants.m_elevatorMotors);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    RobotState.updateElevatorState();
    // tune setpoints on proper field
    switch (RobotState.getCurrentElevatorState()) {
      case Home:
        seekPosition(0);
        break;
      case Intake:
        seekPosition(0.5);
        break;
      case L1:
        seekPosition(l1Position.get());
        break;
      case L2:
        seekPosition(l2Position.get());
        break;
      case L3:
        seekPosition(l3Position.get());
        break;
      case L4:
        seekPosition(l4Position.get());
        break;
      case L4Force:
        seekPosition(l4Position.get());
        break;
      case L4Score:
        seekPosition(l4ScorePosition.get());
        break;
    }

    if (m_elevatorSim.getPositionMeters() == 0) {
      maxSim.setPosition(0);
    }

    m_elevatorSim.setInput(maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    m_elevatorSim.update(0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    maxSim.iterate(
        ElevatorMath.convertDistanceToRotations(
                Meters.of(m_elevatorSim.getVelocityMetersPerSecond()))
            .per(Second)
            .in(RPM),
        RoboRioSim.getVInVoltage(),
        0.020);

    inputs.positionMeters = m_elevatorSim.getPositionMeters();
    inputs.appliedVolts = maxSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    inputs.motorRotations = maxSim.getPosition();
    inputs.velocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.currentAmps = new double[] {m_elevatorSim.getCurrentDrawAmps()};
    inputs.state = RobotState.getCurrentElevatorState();
  }

  @Override
  public void setVoltage(double volts) {
    m_elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void seekPosition(double position) {
    double ff = ElevatorConstants.kGSim + ElevatorConstants.kVSim * (maxSim.getVelocity() / 60);
    elevatorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    Logger.recordOutput("Elevator/Setpoint", position);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setElevatorState(ElevatorState state) {
    RobotState.setQueuedElevatorState(state);
  }
}
