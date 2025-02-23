package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private Pose3d finalElevatorPose = new Pose3d();

  public Elevator(ElevatorIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    finalElevatorPose = new Pose3d(0.11, -0.13, 0.16 + inputs.positionMeters, new Rotation3d());
    Pose3d zeroedElevatorPose = new Pose3d(0, 0, 0, new Rotation3d());
    Logger.recordOutput("Elevator/Zeroed Pose", zeroedElevatorPose);
    Logger.recordOutput("Elevator/Final Pose", finalElevatorPose);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void setElevatorState(ElevatorState state) {
    io.setElevatorState(state);
  }

  public void resetElevatorEncoder() {
    io.resetElevatorEncoder();
  }

  // Todo: add methods, add elevator poses, figure out state logic

}
