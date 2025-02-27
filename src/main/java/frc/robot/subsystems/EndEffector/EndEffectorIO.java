package frc.robot.subsystems.EndEffector;

import frc.robot.RobotState.EndEffectorState;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public double velocityMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public EndEffectorState state = EndEffectorState.Stopped;
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setEndEffectorState(EndEffectorState state) {}
}
