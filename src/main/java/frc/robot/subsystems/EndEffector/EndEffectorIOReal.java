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
  private LoggedTunableNumber intakeTouchTime =
      new LoggedTunableNumber("EndEffector/intake touch time", 0.2);
  private LoggedTunableNumber l1LeftSpeed = new LoggedTunableNumber("EndEffector/L1Left", 4);
  private LoggedTunableNumber l1RightSpeed = new LoggedTunableNumber("EndEffector/L1Right", -1);

  private LoggedTunableNumber jamCurrentSpike =
      new LoggedTunableNumber("EndEffector/jam current spike", 30);
  private LoggedTunableNumber jamTime = new LoggedTunableNumber("EndEffector/jam time", 0.5);

  double highCurrentStartTime = 0;
  double jamCurrentStartTime = 0;

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

    // current limit sensing if canrange disconnected
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
      } else if (leftMotor.getOutputCurrent() >= currentSpike.get()
          || rightMotor.getOutputCurrent() >= currentSpike.get()
              && RobotState.getEndEffectorState() == EndEffectorState.Intake) {
        Logger.recordOutput("EndEffector/Current Spike Active", true);
        if (highCurrentStartTime == 0) {
          highCurrentStartTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - highCurrentStartTime >= intakeTouchTime.get()) {
          if (RobotState.getCoralState() != CoralState.HasCoral) {
            RobotState.setCoralState(CoralState.CoralTouchedIntake);
          }
        } else {
          highCurrentStartTime = 0;
          Logger.recordOutput("EndEffector/Current Spike Active", false);
        }
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
        if (RobotState.getCoralState() == CoralState.NoCoral
            || RobotState.getCoralState() == CoralState.CoralTouchedIntake) {
          setVoltage(4.0);
        } else {
          setEndEffectorState(EndEffectorState.Stopped);
          stop();
        }

        // if intake jams, reverse until no current spike, then go back to intake
        if (leftMotor.getOutputCurrent() >= jamCurrentSpike.get()
            || rightMotor.getOutputCurrent() >= jamCurrentSpike.get()) {
          Logger.recordOutput("EndEffector/Jam Current Spike Active", true);
          if (jamCurrentStartTime == 0) {
            jamCurrentStartTime = Timer.getFPGATimestamp();
          } else if (Timer.getFPGATimestamp() - jamCurrentStartTime >= jamTime.get()) {
            RobotState.setEndEffectorState(EndEffectorState.Reverse);
          }
        } else {
          jamCurrentStartTime = 0;
          RobotState.setEndEffectorState(EndEffectorState.Intake);
          Logger.recordOutput("EndEffector/Jam Current Spike Active", false);
        }

        Logger.recordOutput(
            "EndEffector/Jam Current Spike Time",
            leftMotor.getOutputCurrent() >= jamCurrentSpike.get()
                    || rightMotor.getOutputCurrent() >= jamCurrentSpike.get()
                        && RobotState.getEndEffectorState() == EndEffectorState.Intake
                ? Timer.getFPGATimestamp() - jamCurrentStartTime
                : jamCurrentStartTime);

        break;
      case Score:
        if (RobotState.getCurrentElevatorState() == ElevatorState.L1) {
          leftMotor.setVoltage(l1LeftSpeed.get());
          rightMotor.setVoltage(l1RightSpeed.get());
          // setVoltage(3);
        } else if (RobotState.getCurrentElevatorState() == ElevatorState.L2
            || RobotState.getCurrentElevatorState() == ElevatorState.Home) {
          setVoltage(3);

        } else if (RobotState.getCurrentElevatorState() == ElevatorState.L3) {
          setVoltage(2.5);
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
