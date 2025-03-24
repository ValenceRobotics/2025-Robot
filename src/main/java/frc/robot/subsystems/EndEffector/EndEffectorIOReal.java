package frc.robot.subsystems.EndEffector;

import static frc.robot.util.SparkUtil.*;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.ColorPeriod;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;
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
  Canandcolor canandcolor = new Canandcolor(13);

  private LoggedTunableNumber l4ScoreSpeed = new LoggedTunableNumber("EndEffector/L4Score", 3);
  private LoggedTunableNumber currentSpike =
      new LoggedTunableNumber("EndEffector/current spike", 15);
  private LoggedTunableNumber spikeTime = new LoggedTunableNumber("EndEffector/spike time", 0.1);

  double highCurrentStartTime = 0;

  public EndEffectorIOReal() {

    // Instantiate an empty settings object.
    CanandcolorSettings settings = canandcolor.getSettings();

    // Sets all frame periods to 0.5 seconds to achieve less than 0.1% total CANbus utilization
    settings.setColorFramePeriod(0.5);
    settings.setProximityFramePeriod(0.010);
    settings.setDigoutFramePeriod(0.5);
    settings.setStatusFramePeriod(1);

    settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
    settings.setColorIntegrationPeriod(ColorPeriod.k25ms);

    // Ensure that proximity and color frames are not sent out automatically when the sensor data
    // updates
    settings.setAlignProximityFramesToIntegrationPeriod(true);
    settings.setAlignColorFramesToIntegrationPeriod(false);

    // Sets digital output port 1 to use the digital logic system, setting it to normally closed
    // settings.setDigoutPinConfig(
    //     canandcolor.digout1().channelIndex(), DigoutPinConfig.kDigoutLogicActiveHigh);

    // // Sets the digout frame trigger to send when the digout goes high or low
    // settings.setDigoutFrameTrigger(
    //     canandcolor.digout1().channelIndex(), DigoutFrameTrigger.kRisingAndFalling);

    // Save settings to device
    canandcolor.setSettings(settings);

    // canandcolor
    //     .digout1()
    //     .configureSlots(new HSVDigoutConfig().setMaxProximity(0.06).setMinProximity(0));

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
    Logger.recordOutput("EndEffector/Proximity Sensor Connected", canandcolor.isConnected());
    // Logger.recordOutput("EndEffector/CanandColor Digout Value",
    // canandcolor.digout1().getValue());

    if (!canandcolor.isConnected()) {
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
      if (canandcolor.getProximity() <= 0.06) {
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
