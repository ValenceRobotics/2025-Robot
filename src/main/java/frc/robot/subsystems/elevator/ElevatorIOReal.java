package frc.robot.subsystems.elevator;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.RobotState.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {

  private SparkMax elevatorMaster = new SparkMax(ElevatorConstants.motorId1, MotorType.kBrushless);
  private SparkMax elevatorSlave = new SparkMax(ElevatorConstants.motorId2, MotorType.kBrushless);
  private SparkClosedLoopController elevatorController;
  private DigitalInput revLimitSwitch = new DigitalInput(9);

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
        .pid(ElevatorConstants.kPReal.get(), 0, ElevatorConstants.kDReal.get())
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
    elevatorSlaveConfig.follow(elevatorMaster, false);

    Logger.recordOutput("Elevator/MaxVelocity", ElevatorConstants.maxVelocity);
    Logger.recordOutput("Elevator/MaxAcceleration", ElevatorConstants.maxAcceleration);
    SmartDashboard.putNumber("hi", 1);

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

    if (!getLimitSwitchState()) {
      resetElevatorEncoder();
    }

    RobotState.updateElevatorState();
    // tune setpoints on proper field
    switch (RobotState.getCurrentElevatorState()) {
      case Home:
        seekPosition(0);
        break;
      case L1:
        seekPosition(5.5);
        break;
      case L2:
        seekPosition(8.8);
        break;
      case L3:
        seekPosition(18);
        break;
      case L4:
        seekPosition(32);
        break;
      case testing:
        seekPosition(SmartDashboard.getNumber("hi", 0));
    }

    Logger.recordOutput("Elevator/Limit Switch", getLimitSwitchState());
    inputs.appliedVolts = elevatorMaster.getBusVoltage();
    inputs.motorRotations = elevatorMaster.getEncoder().getPosition();
    inputs.velocityRotationsPerSec = elevatorMaster.getEncoder().getVelocity();
    inputs.currentAmps = new double[] {elevatorMaster.getOutputCurrent()};
    inputs.state = RobotState.getCurrentElevatorState();
  }

  public boolean getLimitSwitchState() {
    return revLimitSwitch.get();
  }

  @Override
  public void setVoltage(double volts) {
    elevatorMaster.setVoltage(volts);
  }

  @Override
  public void seekPosition(double position) {
    double ff = ElevatorConstants.kGReal.get();
    // + ElevatorConstants.kVReal * (elevatorMaster.getEncoder().getVelocity() / 60);
    elevatorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    Logger.recordOutput("Elevator/Setpoint", position);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void resetElevatorEncoder() {
    elevatorMaster.getEncoder().setPosition(0);
  }

  @Override
  public void setElevatorState(ElevatorState state) {
    RobotState.setQueuedElevatorState(state);
  }
}
