package frc.robot.subsystems.elevator;

import frc.robot.RobotState.ElevatorState;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double motorRotations = 0.0;
    public double encoderRotations = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public ElevatorState state = ElevatorState.Home;
    public boolean limitSwitch = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void seekPosition(double position) {}

  public default void stop() {}

  public default void setElevatorState(ElevatorState state) {}

  public default void resetElevatorEncoder() {}
}
