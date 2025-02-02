package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
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
import frc.robot.RobotState.ElevatorState;
import frc.robot.util.ElevatorMath;
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

  private ElevatorState currentState = ElevatorState.Home;

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
        .pid(ElevatorConstants.kP, 0, ElevatorConstants.kD)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(ElevatorConstants.maxVelocity)
        .maxAcceleration(ElevatorConstants.maxAcceleration)
        .allowedClosedLoopError(ElevatorConstants.allowedError);

    max.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    maxSim = new SparkMaxSim(max, ElevatorConstants.m_elevatorMotors);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    switch (currentState) {
      case Home:
        seekPosition(0);
        break;
      case L2:
        seekPosition(0.2286);
        break;
      case L3:
        seekPosition(0.6096);
        break;
      case L4:
        seekPosition(1.3);
        break;
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
    inputs.state = currentState;
  }

  @Override
  public void setVoltage(double volts) {
    m_elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void seekPosition(double position) {
    elevatorController.setReference(
        ElevatorMath.convertDistanceToRotations(Meters.of(position)).in(Rotations),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
    Logger.recordOutput(
        "Elevator/Setpoint",
        ElevatorMath.convertDistanceToRotations(Meters.of(position)).in(Rotations));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setElevatorState(ElevatorState state) {
    currentState = state;
  }
}
