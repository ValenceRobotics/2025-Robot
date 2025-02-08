package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.util.SparkUtil.*;


import frc.robot.RobotState;
import frc.robot.RobotState.EndEffectorState;

public class EndEffectorIOReal implements EndEffectorIO {

  SparkMax leftMotor = new SparkMax(EndEffectorConstants.leftMotorId, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(EndEffectorConstants.rightMotorId, MotorType.kBrushless);

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
        leftMotor.configure(endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        );
    
    tryUntilOk(
      rightMotor,
      5,
      () ->
      rightMotor.configure(endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
      );
  
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {

    switch(RobotState.getEndEffectorState()) {
      case Score:
        setVoltage(-3.0);
        break;
      case ScoreL1:
        leftMotor.setVoltage(-3.0);
        rightMotor.setVoltage(1.0);
      case Reverse:
        setVoltage(3.0);
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
