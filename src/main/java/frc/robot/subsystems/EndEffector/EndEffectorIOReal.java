package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class EndEffectorIOReal implements EndEffectorIO {
  private final EndEffectorIO.EndEffectorIOInputs inputs = new EndEffectorIO.EndEffectorIOInputs();

  SparkMax leftMotor = new SparkMax(9, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(10, MotorType.kBrushless);

  public EndEffectorIOReal() {}

  @Override
  public void updateInputs(EndEffectorIO.EndEffectorIOInputs inputs) {
    this.inputs.velocityMetersPerSec = inputs.velocityMetersPerSec;
    this.inputs.appliedVolts = inputs.appliedVolts;
    this.inputs.currentAmps = inputs.currentAmps;
  }

  @Override
  public void setVoltage(double volts) {
    inputs.appliedVolts = volts;
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(-volts);
  }
}
