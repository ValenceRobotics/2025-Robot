package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.SparkUtil.*;

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
import frc.robot.RobotState;
import frc.robot.RobotState.ElevatorState;
import frc.robot.util.ElevatorMath;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

  private SparkMax elevatorMaster = new SparkMax(ElevatorConstants.motorId1, MotorType.kBrushless);
  private SparkMax elevatorSlave = new SparkMax(ElevatorConstants.motorId2, MotorType.kBrushless);
  private SparkClosedLoopController elevatorController;

  public ElevatorIOReal() {
    elevatorController = elevatorMaster.getClosedLoopController();
    var elevatorConfig = new SparkMaxConfig();

    elevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12.0);
    elevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kPReal, 0, ElevatorConstants.kDReal)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(ElevatorConstants.maxVelocity)
        .maxAcceleration(ElevatorConstants.maxAcceleration)
        .allowedClosedLoopError(ElevatorConstants.allowedError);

    var elevatorSlaveConfig = new SparkMaxConfig();
    elevatorSlaveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12.0);
    elevatorSlaveConfig.follow(elevatorMaster, true);

    Logger.recordOutput("Elevator/MaxVelocity", ElevatorConstants.maxVelocity);
    Logger.recordOutput("Elevator/MaxAcceleration", ElevatorConstants.maxAcceleration);

    tryUntilOk(
        elevatorMaster,
        5,
        () ->
            elevatorMaster.configure(
                elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        elevatorSlave,
        5,
        () ->
            elevatorSlave.configure(
                elevatorSlaveConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // might want to switcht to just raw rotations

    RobotState.updateElevatorState();

    switch (RobotState.getCurrentElevatorState()) {
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

    // TODO: rev limit switch zeroing

    inputs.positionMeters = elevatorMaster.getEncoder().getPosition(); // convert to meters
    inputs.appliedVolts = elevatorMaster.getBusVoltage();
    inputs.motorRotations = elevatorMaster.getEncoder().getPosition();
    inputs.velocityMetersPerSec =
        elevatorMaster.getEncoder().getVelocity(); // convert to meters per second
    inputs.currentAmps = new double[] {elevatorMaster.getOutputCurrent()};
    inputs.state = RobotState.getCurrentElevatorState();
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMaster.setVoltage(volts);
  }

  @Override
  public void seekPosition(double position) {
    double ff =
        ElevatorConstants.kGReal
            + ElevatorConstants.kVReal * (elevatorMaster.getEncoder().getVelocity() / 60);
    elevatorController.setReference(
        ElevatorMath.convertDistanceToRotations(Meters.of(position)).in(Rotations),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff);
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
    RobotState.setQueuedElevatorState(state);
  }
}
