package frc.robot.subsystems.algae;

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
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AlgaeIOReal implements AlgaeIO {

  SparkMax pivotMotor = new SparkMax(AlgaeConstants.pivotMotorId, MotorType.kBrushless);
  SparkMax shooterMotor = new SparkMax(AlgaeConstants.shooterMotorId, MotorType.kBrushless);
  SparkClosedLoopController pivotController;

  private LoggedTunableNumber kg = new LoggedTunableNumber("Algae/Pivot Kg", 0);
  private LoggedTunableNumber stowPosition = new LoggedTunableNumber("Algae/Pivot Stow", 0);

  private LoggedTunableNumber activePosition = new LoggedTunableNumber("Algae/Active position", 0);
  private LoggedTunableNumber intakeStallCurrent =
      new LoggedTunableNumber("Algae/Stall Current", 30);

  public AlgaeIOReal() {

    pivotController = pivotMotor.getClosedLoopController();
    var pivotConfig = new SparkMaxConfig();
    var shooterConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(AlgaeConstants.pivotKp, 0, AlgaeConstants.pivotKd)
        .outputRange(-1, 1);

    shooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);

    tryUntilOk(
        pivotMotor,
        5,
        () ->
            pivotMotor.configure(
                pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        shooterMotor,
        5,
        () ->
            shooterMotor.configure(
                shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

    // switch (RobotState.getAlgaeState()) {
    //   case Stow:
    //     setShooterVoltage(0);
    //     setPivotPosition(stowPosition.get());
    //     break;
    //   case Intake:
    //     if (shooterMotor.getOutputCurrent() >= intakeStallCurrent.get()) {
    //       setShooterVoltage(-0.5);
    //     } else {
    //       setShooterVoltage(-3);
    //     }
    //     setPivotPosition(activePosition.get());
    //   case Shoot:
    //     setShooterVoltage(4);
    //     setPivotPosition(activePosition.get());
    // }

    inputs.pivotAppliedVolts = pivotMotor.getBusVoltage();
    inputs.shooterAppliedVolts = shooterMotor.getBusVoltage();
    inputs.positionRotations = (pivotMotor.getEncoder().getPosition() / 15) + 0;
    inputs.positionRad = (2 * Math.PI * ((pivotMotor.getEncoder().getPosition() / 15) + 0));
    inputs.pivotCurrentAmps = pivotMotor.getOutputCurrent();
    inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
    inputs.state = RobotState.getAlgaeState();
  }

  @Override
  public void setPivotPosition(double position) {
    double ff =
        kg.get()
            * Math.cos(
                2
                    * Math.PI
                    * ((pivotMotor.getEncoder().getPosition() / 15)
                        + 0)); // tune angle offset and kg
    pivotController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
    Logger.recordOutput("Algae/Pivot Setpoint", position);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooterMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    pivotMotor.set(0);
    shooterMotor.set(0);
  }
}
