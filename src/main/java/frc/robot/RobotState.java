package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static RobotState instance = new RobotState();

  // State enums
  public enum ElevatorState {
    Home,
    Intake,
    L1,
    L2,
    L3,
    L4,
    L4Score,
    L4Force,
    testing
  }

  public enum SingleTagMode {
    Available,
    NotAvailable
  }

  public enum ElevatorSetpoint {
    AtSetpoint,
    NotAtSetpoint
  }

  public enum DriveState {
    Driving,
    Aligned
  }

  public enum AlignState { // extra work fuse with drive state later
    Aligning,
    NotAligning
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

  public enum SystemMode {
    Manual,
    Auto
  }

  // Private state tracking
  private ElevatorState currentElevatorState = ElevatorState.Home;
  private ElevatorState queuedElevatorState = ElevatorState.Home;
  private DriveState previousDriveState = DriveState.Driving;
  private DriveState driveState = DriveState.Driving;
  private CoralState coralState = CoralState.HasCoral; // set no coral later
  private EndEffectorState endEffectorState = EndEffectorState.Stopped;
  private ElevatorSetpoint elevatorSetpointState = ElevatorSetpoint.NotAtSetpoint;
  private SystemMode systemMode = SystemMode.Auto;
  private SingleTagMode singleTagMode = SingleTagMode.NotAvailable;
  private AlignState alignState = AlignState.NotAligning;

  // Singleton accessor
  public static RobotState getInstance() {
    return instance;
  }

  public static void setAlignState(AlignState state) {
    instance.alignState = state;
    Logger.recordOutput("RobotState/AlignState", state.toString());
  }

  public static AlignState getAlignState() {
    return instance.alignState;
  }

  public static void setSingleTagMode(SingleTagMode state) {
    instance.singleTagMode = state;
    Logger.recordOutput("RobotState/Single Tag Mode", state.toString());
  }

  public static SingleTagMode getSingleTagMode() {
    return instance.singleTagMode;
  }

  public static void setSystemMode(SystemMode mode) {
    instance.systemMode = mode;
    Logger.recordOutput("RobotState/SystemMode", mode.toString());
  }

  public static SystemMode getSystemMode() {
    return instance.systemMode;
  }

  public static void setElevatorSetpoint(ElevatorSetpoint state) {
    instance.elevatorSetpointState = state;

    Logger.recordOutput("RobotState/ElevatorSetpointState", state.toString());
  }

  public static ElevatorSetpoint getElevatorSetpoint() {
    return instance.elevatorSetpointState;
  }

  // Elevator state management
  public static void setQueuedElevatorState(ElevatorState state) {
    instance.queuedElevatorState = state;
    if (state == ElevatorState.Home) {
      instance.currentElevatorState = ElevatorState.Home; // Home executes immediately
    } else if (state == ElevatorState.L4Force) {
      instance.currentElevatorState = ElevatorState.L4Force; // L4Force executes immediately
    } else if (state == ElevatorState.Intake) {
      instance.currentElevatorState = ElevatorState.Intake; // Intake executes immediately
    }
    Logger.recordOutput("RobotState/QueuedElevatorState", state.toString());
  }

  public static ElevatorState getCurrentElevatorState() {
    return instance.currentElevatorState;
  }

  public static void updateElevatorState() {
    if (instance.currentElevatorState == ElevatorState.Intake
        && instance.coralState == CoralState.HasCoral) {
      setQueuedElevatorState(ElevatorState.Home);
    }
    if (canExecuteQueuedState()) {
      instance.currentElevatorState = instance.queuedElevatorState;
      Logger.recordOutput(
          "RobotState/CurrentElevatorState", instance.currentElevatorState.toString());
    }
  }

  // Drive state management
  public static void setDriveState(DriveState state) {
    // Check for transition from CloseToAlign to Driving
    // if (instance.previousDriveState == DriveState.Aligned && state == DriveState.Driving) {
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
    if (instance.systemMode == SystemMode.Auto) {
      if (instance.queuedElevatorState == ElevatorState.Home
          || instance.queuedElevatorState == ElevatorState.L4Force
          || instance.queuedElevatorState == ElevatorState.Intake) {
        return true; // Home state has no requirements, force override in l4 auto
      }
      return instance.driveState == DriveState.Aligned
          && instance.coralState == CoralState.HasCoral;
    } else {
      return true;
    }
  }

  public enum TagChoice {
    TAG_6(6),
    TAG_7(7),
    TAG_8(8),
    TAG_9(9),
    TAG_10(10),
    TAG_11(11),
    TAG_17(17),
    TAG_18(18),
    TAG_19(19),
    TAG_20(20),
    TAG_21(21),
    TAG_22(22),
    NONE(0);

    private final int id;

    TagChoice(int id) {
      this.id = id;
    }

    public int getId() {
      return id;
    }

    public static TagChoice fromId(int id) {
      for (TagChoice tag : values()) {
        if (tag.getId() == id) {
          return tag;
        }
      }
      return NONE;
    }
  }

  // Add to existing fields
  private TagChoice currentTag = TagChoice.NONE;

  // Add getters/setters
  public static void setCurrentTag(TagChoice tag) {
    instance.currentTag = tag;
    Logger.recordOutput("RobotState/CurrentTag", tag.toString());
  }

  public static TagChoice getCurrentTag() {
    return instance.currentTag;
  }

  // Helper method to get tag ID from ReefTags
  public static void updateTagFromSide(DriveConstants.ReefTags side) {
    if (side == null) {
      setCurrentTag(TagChoice.NONE);
      return;
    }

    Optional<Pose3d> tagPose = side.getPose();
    if (tagPose.isEmpty()) {
      setCurrentTag(TagChoice.NONE);
      return;
    }

    // Find tag ID from pose
    for (AprilTag tag : DriveConstants.aprilTagLayout.getTags()) {
      if (tag.pose.equals(tagPose.get())) {
        setCurrentTag(TagChoice.fromId(tag.ID));
        return;
      }
    }
  }

  // Private constructor for singleton
  private RobotState() {}
}
