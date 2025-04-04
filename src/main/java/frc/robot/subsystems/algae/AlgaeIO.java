package frc.robot.subsystems.algae;

import frc.robot.RobotState.AlgaeState;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public double positionRotations = 0.0;
    public double positionRad = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double shooterCurrentAmps = 0.0;
    public AlgaeState state = AlgaeState.Stow;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setPivotPosition(double position) {}

  public default void setPivotVoltage(double volts) {}

  public default void setShooterVoltage(double volts) {}

  public default void setAlgaeState(AlgaeState state) {}

  public default void stop() {}
}
