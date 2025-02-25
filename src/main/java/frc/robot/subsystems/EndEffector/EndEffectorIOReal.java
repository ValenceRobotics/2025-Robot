package frc.robot.subsystems.EndEffector;

import static frc.robot.util.SparkUtil.*;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralState;
import frc.robot.RobotState.ElevatorState;
import frc.robot.RobotState.EndEffectorState;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOReal implements EndEffectorIO {

  SparkMax leftMotor = new SparkMax(EndEffectorConstants.leftMotorId, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(EndEffectorConstants.rightMotorId, MotorType.kBrushless);
  Canandcolor canandcolor = new Canandcolor(13);

  public EndEffectorIOReal() {
    var endEffectorConfig = new SparkMaxConfig();

    endEffectorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(EndEffectorConstants.currentLimit)
        .voltageCompensation(12.0);

    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {

    Logger.recordOutput("EndEffector/Proximity", canandcolor.getProximity());

    if (canandcolor.getProximity() <= 0.06) {
      RobotState.setCoralState(CoralState.HasCoral);
    } else {
      RobotState.setCoralState(CoralState.NoCoral);
    }

    switch (RobotState.getEndEffectorState()) {
      case Intake:
        if (RobotState.getCoralState() == CoralState.NoCoral) {
          setVoltage(4.0);
        } else {
          setEndEffectorState(EndEffectorState.Stopped);
        }
        break;
      case Score:
        if (RobotState.getCurrentElevatorState() == ElevatorState.L1) {
          leftMotor.setVoltage(2);
          rightMotor.setVoltage(-0.2);
          // setVoltage(3);
        } else if (RobotState.getCurrentElevatorState() == ElevatorState.L2
            || RobotState.getCurrentElevatorState() == ElevatorState.L3) {
          setVoltage(8);
        } else {
          setVoltage(3);
        }
        break;
      case Reverse:
        setVoltage(-3.0);
        break;
      case Stopped:
        stop();
        break;
    }

    inputs.velocityMetersPerSec = inputs.velocityMetersPerSec;
    inputs.leftAppliedVolts = leftMotor.getBusVoltage();
    inputs.rightAppliedVolts = rightMotor.getBusVoltage();
    inputs.currentAmps = inputs.currentAmps;
    inputs.state = RobotState.getEndEffectorState();
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(-volts);
  }

  @Override
  public void stop() {
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
  }

  @Override
  public void setEndEffectorState(EndEffectorState state) {
    RobotState.setEndEffectorState(state);
  }
}
