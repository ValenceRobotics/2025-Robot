package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    testing,
    AlgaeDescoreCheck,
    LowAlgaeDescore,
    HighAlgaeDescore
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
    CloseToAlign,
    Aligned
  }

  public enum AlignState {
    InAlignRange,
    NotInAlignRange
  }

  public enum CoralState {
    HasCoral,
    CoralTouchedIntake,
    NoCoral
  }

  public enum EndEffectorState {
    Intake,
    Score,
    Stopped,
    Reverse
  }

  public enum AlgaeState {
    Stow,
    Intake,
    Shoot
  }

  public enum SystemMode {
    Manual,
    Auto
  }

  public enum AimbotMode {
    NoAimbot,
    Aimbot
  }

  // Private state tracking
  private ElevatorState currentElevatorState = ElevatorState.Home;
  private ElevatorState queuedElevatorState = ElevatorState.Home;
  private DriveState previousDriveState = DriveState.Driving;
  private DriveState driveState = DriveState.Driving;
  private CoralState coralState = CoralState.NoCoral; // set no coral later
  private EndEffectorState endEffectorState = EndEffectorState.Stopped;
  private ElevatorSetpoint elevatorSetpointState = ElevatorSetpoint.NotAtSetpoint;
  private SystemMode systemMode = SystemMode.Auto;
  private SingleTagMode singleTagMode = SingleTagMode.NotAvailable;
  private AlignState alignState = AlignState.NotInAlignRange;
  private AlgaeState algaeState = AlgaeState.Stow;
  private AimbotMode aimbotMode = AimbotMode.Aimbot;

  // Singleton accessor
  public static RobotState getInstance() {
    return instance;
  }

  public static void setAimbotMode(AimbotMode mode) {
    instance.aimbotMode = mode;
    Logger.recordOutput("RobotState/Aimbot Mode", mode.toString());
  }

  public static AimbotMode getAimbotMode() {
    return instance.aimbotMode;
  }

  public static void setAlgaeState(AlgaeState state) {
    instance.algaeState = state;
    Logger.recordOutput("RobotState/AlgaeState", state.toString());
  }

  public static AlgaeState getAlgaeState() {
    return instance.algaeState;
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
      } else {
        return instance.driveState == DriveState.Aligned
            && instance.coralState == CoralState.HasCoral;
      }
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

    public Pose2d getPose() {
      if (this == NONE) {
        return new Pose2d();
      }
      return DriveConstants.aprilTagLayout
          .getTagPose(this.id)
          .get()
          .toPose2d()
          .plus(
              new Transform2d(
                  DriveConstants.trackWidth / 2 + DriveConstants.bumperThickness,
                  0,
                  new Rotation2d(Math.PI)));
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
