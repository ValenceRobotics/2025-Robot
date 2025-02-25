package frc.robot;

import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance = new RobotState();

  // State enums
  public enum ElevatorState {
    Home,
    L1,
    L2,
    L3,
    L4, 
    testing
  }

  public enum DriveState {
    Driving,
    Aligning,
    CloseToAlign
  }

  public enum CoralState {
    HasCoral,
    NoCoral
  }

  public enum EndEffectorState {
    Intake,
    Score,
    Stopped,
    Reverse
  }

  // Private state tracking
  private ElevatorState currentElevatorState = ElevatorState.Home;
  private ElevatorState queuedElevatorState = ElevatorState.Home;
  private DriveState previousDriveState = DriveState.Driving;
  private DriveState driveState = DriveState.Driving;
  private CoralState coralState = CoralState.HasCoral; // set no coral later
  private EndEffectorState endEffectorState = EndEffectorState.Stopped;

  // Singleton accessor
  public static RobotState getInstance() {
    return instance;
  }

  // Elevator state management
  public static void setQueuedElevatorState(ElevatorState state) {
    instance.queuedElevatorState = state;
    if (state == ElevatorState.Home) {
      instance.currentElevatorState = ElevatorState.Home; // Home executes immediately
    }
    Logger.recordOutput("RobotState/QueuedElevatorState", state.toString());
  }

  public static ElevatorState getCurrentElevatorState() {
    return instance.currentElevatorState;
  }

  public static void updateElevatorState() {
    if (canExecuteQueuedState()) {
      instance.currentElevatorState = instance.queuedElevatorState;
      Logger.recordOutput(
          "RobotState/CurrentElevatorState", instance.currentElevatorState.toString());
    }
  }

  // Drive state management
  public static void setDriveState(DriveState state) {
    // Check for transition from CloseToAlign to Driving
    // if (instance.previousDriveState == DriveState.CloseToAlign && state == DriveState.Driving) {
    //   setQueuedElevatorState(ElevatorState.Home);
    // }

    instance.previousDriveState = instance.driveState;
    instance.driveState = state;
    Logger.recordOutput("RobotState/PreviousDriveState", instance.previousDriveState.toString());
    Logger.recordOutput("RobotState/DriveState", state.toString());
  }

  public static DriveState getDriveState() {
    return instance.driveState;
  }

  // Coral state management
  public static void setCoralState(CoralState state) {
    instance.coralState = state;

    Logger.recordOutput("RobotState/CoralState", state.toString());
  }

  public static CoralState getCoralState() {
    return instance.coralState;
  }

  // EndEffector state management
  public static void setEndEffectorState(EndEffectorState state) {
    instance.endEffectorState = state;
    Logger.recordOutput("RobotState/EndEffectorState", state.toString());
  }

  public static EndEffectorState getEndEffectorState() {
    return instance.endEffectorState;
  }

  // Elevator state validation
  private static boolean canExecuteQueuedState() {
    return true;
    //   if (instance.queuedElevatorState == ElevatorState.Home) {
    //     return true; // Home state has no requirements
    //   }
    //   return instance.driveState == DriveState.CloseToAlign
    //       && instance.coralState == CoralState.HasCoral;
    // }
  }

  // Private constructor for singleton
  private RobotState() {}
}
