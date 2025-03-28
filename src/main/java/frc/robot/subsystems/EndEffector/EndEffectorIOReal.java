package frc.robot.subsystems.EndEffector;

import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.RobotState.CoralState;
import frc.robot.RobotState.ElevatorState;
import frc.robot.RobotState.EndEffectorState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOReal implements EndEffectorIO {

  SparkMax leftMotor = new SparkMax(EndEffectorConstants.leftMotorId, MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(EndEffectorConstants.rightMotorId, MotorType.kBrushless);

  CANrange canRange = new CANrange(13);

  private LoggedTunableNumber l4ScoreSpeed = new LoggedTunableNumber("EndEffector/L4Score", 3);
  private LoggedTunableNumber currentSpike =
      new LoggedTunableNumber("EndEffector/current spike", 15);
  private LoggedTunableNumber spikeTime = new LoggedTunableNumber("EndEffector/spike time", 0.1);

  double highCurrentStartTime = 0;

  public EndEffectorIOReal() {

    CANrangeConfiguration configs = new CANrangeConfiguration();

    canRange.getConfigurator().apply(configs);

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

    Logger.recordOutput("EndEffector/Proximity", canRange.getDistance().getValueAsDouble());
    Logger.recordOutput("EndEffector/Proximity Sensor Connected", canRange.isConnected());
    // Logger.recordOutput("EndEffector/CanandColor Digout Value",
    // canandcolor.digout1().getValue());

    if (!canRange.isConnected()) {
      if (leftMotor.getOutputCurrent() >= currentSpike.get()
          || rightMotor.getOutputCurrent() >= currentSpike.get()
              && RobotState.getEndEffectorState() == EndEffectorState.Intake) {
        Logger.recordOutput("EndEffector/Current Spike Active", true);
        if (highCurrentStartTime == 0) {
          highCurrentStartTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - highCurrentStartTime >= spikeTime.get()) {
          RobotState.setCoralState(CoralState.HasCoral);
        }
      } else {
        highCurrentStartTime = 0;
        RobotState.setCoralState(CoralState.NoCoral);
        Logger.recordOutput("EndEffector/Current Spike Active", false);
      }
    } else {
      if (canRange.getDistance().getValueAsDouble() <= 0.06) {
        RobotState.setCoralState(CoralState.HasCoral);
      } else {
        RobotState.setCoralState(CoralState.NoCoral);
      }
    }

    Logger.recordOutput(
        "EndEffector/Current Spike Time",
        leftMotor.getOutputCurrent() >= currentSpike.get()
                || rightMotor.getOutputCurrent() >= currentSpike.get()
                    && RobotState.getEndEffectorState() == EndEffectorState.Intake
            ? Timer.getFPGATimestamp() - highCurrentStartTime
            : highCurrentStartTime);

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
          leftMotor.setVoltage(3);
          rightMotor.setVoltage(-0.5);
          // setVoltage(3);
        } else if (RobotState.getCurrentElevatorState() == ElevatorState.L2
            || RobotState.getCurrentElevatorState() == ElevatorState.L3
            || RobotState.getCurrentElevatorState() == ElevatorState.Home) {
          setVoltage(3);
        } else {
          setVoltage(l4ScoreSpeed.get());
          // new WaitCommand(0.1);
          RobotState.setQueuedElevatorState(ElevatorState.L4Score);
        }
        break;
      case Reverse:
        setVoltage(-1.5);
        break;
      case Stopped:
        stop();
        break;
    }

    inputs.velocityMetersPerSec = inputs.velocityMetersPerSec;
    inputs.leftAppliedVolts = leftMotor.getBusVoltage();
    inputs.rightAppliedVolts = rightMotor.getBusVoltage();
    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
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
