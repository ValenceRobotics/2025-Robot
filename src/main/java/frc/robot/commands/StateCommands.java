package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotState.*;

public class StateCommands {

  private StateCommands() {}
  // Main mechanism state setter with raw enums
  public static Command setMechanismState(Enum<?>... states) {
    return Commands.runOnce(
        () -> {
          for (Enum<?> state : states) {
            if (state instanceof ElevatorState) {
              RobotState.setQueuedElevatorState((ElevatorState) state);
            } else if (state instanceof DriveState) {
              RobotState.setDriveState((DriveState) state);
            } else if (state instanceof EndEffectorState) {
              RobotState.setEndEffectorState((EndEffectorState) state);
            } else if (state instanceof SystemMode) {
              RobotState.setSystemMode((SystemMode) state);
            }
          }
        });
  }
}
