// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.RobotState.AimbotMode;
import frc.robot.RobotState.AlignState;
import frc.robot.RobotState.DriveState;
import frc.robot.RobotState.SingleTagMode;
import frc.robot.subsystems.drive.DriveConstants.CoralScoreLocation;
import frc.robot.subsystems.drive.DriveConstants.HPTags;
import frc.robot.subsystems.drive.DriveConstants.ReefTags;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // private final SwerveSetpointGenerator setpointGenerator;
  // private SwerveSetpoint previousSetpoint;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final SwerveDrivePoseEstimator alignPoseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  Pose2d[] scoreLocations = {new Pose2d(), new Pose2d()};

  private Pose2d closestHPTag = new Pose2d();

  Pose2d verificationPose = new Pose2d();

  ReefTags closestReefTag = null;

  boolean aimbotTrigger = false;

  boolean joysticksActive = false;

  Pose2d bargeShotPose = new Pose2d();

  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // setpointGenerator = new SwerveSetpointGenerator(ppConfig, Units.rotationsToRadians(10.0));
    // previousSetpoint =
    //     new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Initialize ReefTags and CoralScoreLocation with alliance value

    ReefTags.initializeAlliance(
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    CoralScoreLocation.initializeAlliance(
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);
    HPTags.initializeAlliance(
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue);

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      alignPoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    scoreLocations = findClosestReefTag(getPose());
    Logger.recordOutput("Alignment/ScoringLocations", scoreLocations);

    Logger.recordOutput("Alignment/Alliance", DriverStation.getAlliance().orElse(Alliance.Blue));

    closestHPTag =
        getPose().nearest(List.of(HPTags.LEFT_STATION.getPose(), HPTags.RIGHT_STATION.getPose()));

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    if (closestReefTag != null) {
      if (getPose().minus(closestReefTag.getPose().get().toPose2d()).getTranslation().getNorm()
          <= (Units.inchesToMeters(36))) {
        RobotState.setAlignState(AlignState.InAlignRange);
      } else {
        RobotState.setAlignState(AlignState.NotInAlignRange);
      }
    }

    if (RobotState.getSingleTagMode() == SingleTagMode.NotAvailable) {
      verificationPose = getPose();
    } else {
      verificationPose = getSingleTagPose();
    }

    if (verificationPose.minus(getScoreLocations()[0]).getTranslation().getNorm() <= 0.015
        || verificationPose.minus(getScoreLocations()[1]).getTranslation().getNorm() <= 0.015) {
      RobotState.setDriveState(DriveState.Aligned);
    } else if ((verificationPose.minus(getScoreLocations()[0]).getTranslation().getNorm() <= 0.2
        || verificationPose.minus(getScoreLocations()[1]).getTranslation().getNorm() <= 0.2)) {
      RobotState.setDriveState(DriveState.CloseToAlign);
    } else {
      RobotState.setDriveState(DriveState.Driving);
    }

    if (RobotModeTriggers.teleop().getAsBoolean()) {
      aimbotTrigger = RobotState.getAimbotMode() == AimbotMode.Aimbot;
    } else {
      aimbotTrigger = false;
    }

    Logger.recordOutput("Drive/Aimbot Trigger", aimbotTrigger);

    // Log odometry
    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Odometry/SingleTagPose", getSingleTagPose());
    Logger.recordOutput("Odometry/VerificationPose", verificationPose);
    Logger.recordOutput("Odometry/RawGyroRotation", rawGyroRotation);
    Logger.recordOutput("Odometry/ClosestHPTag", closestHPTag);
    if (closestReefTag != null) {
      Logger.recordOutput("Alignment/ClosestReefTag", closestReefTag.toString());
    } else {
      Logger.recordOutput("Alignment/ClosestReefTag", "null");
    }
    Logger.recordOutput("Alignment/BargeShotPose", bargeShotPose);
  }

  public boolean getAimbotTrigger() {
    return aimbotTrigger;
  }

  public Pose2d getBargeShotPose() {
    bargeShotPose =
        new Pose2d(
            AllianceFlipUtil.applyX(FieldConstants.startingLineX),
            getPose().getY(),
            AllianceFlipUtil.apply(new Rotation2d()));
    return bargeShotPose;
  }

  public Pose2d getClosestHPTagPose() {
    return closestHPTag;
  }

  public void setJoysticsActive(boolean active) {
    joysticksActive = active;
  }

  public boolean getJoysticksActive(DoubleSupplier x, DoubleSupplier y) {
    Logger.recordOutput(
        "Drive/JoysticsActive", Math.abs(x.getAsDouble()) + Math.abs(y.getAsDouble()) > 0.15);
    return (Math.abs(x.getAsDouble()) + Math.abs(y.getAsDouble())) > 0.15;
  }
  /**
   * Retrieves the array of possible scoring locations on the field.
   *
   * @return An array of Pose2d objects representing the scoring positions
   */
  public Pose2d[] getScoreLocations() {
    return scoreLocations;
  }

  /**
   * Calculates the 2D Euclidean distance between two 2D poses.
   *
   * @param pose1 The first 2D pose
   * @param pose2 The second 2D pose
   * @return The distance between the two poses in the XY plane
   */
  public double calculateDistance(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(
        Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
  }

  /**
   * Finds the closest reef tag to the current robot position and returns its corresponding scoring
   * locations.
   *
   * @param currentPose The current 2D pose (position and orientation) of the robot
   * @return An array of two Pose2d objects representing the scoring locations associated with the
   *     closest reef tag. The array contains two adjacent scoring locations (e.g., A and B for
   *     SIDE_AB). May return null values if no reef tag is found.
   */
  public Pose2d[] findClosestReefTag(Pose2d currentPose) {
    ReefTags closest = null;
    double minDistance = Double.MAX_VALUE;

    for (ReefTags tag : ReefTags.values()) {
      if (tag.getPose().isPresent()) {
        double distance = calculateDistance(currentPose, tag.getPose().get().toPose2d());
        if (distance < minDistance) {
          minDistance = distance;
          closest = tag;
        }
      }
    }
    closestReefTag = closest;
    RobotState.updateTagFromSide(closest);
    Logger.recordOutput("Alignment/ClosestReefTag", closest);

    Pose2d[] scoringLocations = new Pose2d[2];
    if (closest == ReefTags.SIDE_AB) {
      scoringLocations[0] = CoralScoreLocation.A.getPose();
      scoringLocations[1] = CoralScoreLocation.B.getPose();
    } else if (closest == ReefTags.SIDE_CD) {
      scoringLocations[0] = CoralScoreLocation.C.getPose();
      scoringLocations[1] = CoralScoreLocation.D.getPose();
    } else if (closest == ReefTags.SIDE_EF) {
      scoringLocations[0] = CoralScoreLocation.E.getPose();
      scoringLocations[1] = CoralScoreLocation.F.getPose();
    } else if (closest == ReefTags.SIDE_GH) {
      scoringLocations[0] = CoralScoreLocation.G.getPose();
      scoringLocations[1] = CoralScoreLocation.H.getPose();
    } else if (closest == ReefTags.SIDE_IJ) {
      scoringLocations[0] = CoralScoreLocation.I.getPose();
      scoringLocations[1] = CoralScoreLocation.J.getPose();
    } else if (closest == ReefTags.SIDE_KL) {
      scoringLocations[0] = CoralScoreLocation.K.getPose();
      scoringLocations[1] = CoralScoreLocation.L.getPose();
    }
    return scoringLocations;
  }

  @AutoLogOutput(key = "Drive/AimbotTarget") // return as pose2d for easy logging/debugging
  public Pose2d getAimbotTarget() {
    Translation2d currentPose = new Translation2d(getPose().getX(), getPose().getY());
    Translation2d reefCenter = AllianceFlipUtil.apply(FieldConstants.Reef.center);
    Rotation2d targetAngle = reefCenter.minus(currentPose).getAngle();
    if (getPose().minus(getClosestHPTagPose()).getTranslation().getNorm() <= 2.0) {
      return getClosestHPTagPose();
    } else {
      return new Pose2d(getPose().getX(), getPose().getY(), targetAngle);
    }
  }
  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    // SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "Odometry/SingleTagPose")
  public Pose2d getSingleTagPose() {
    return alignPoseEstimator.getEstimatedPosition();
  }

  public Pose2d getReefPose(Pose2d finalPose) {
    var tagPose = getSingleTagPose();
    // Use estimated pose if tag pose is not present
    if (RobotState.getSingleTagMode() == SingleTagMode.NotAvailable) {
      return getPose();
    }

    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (getPose().getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);

    return getPose().interpolate(tagPose, 1.0 - t);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    alignPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    if (RobotState.getAlignState() == AlignState.NotInAlignRange) {
      Logger.recordOutput("Drive/Align Using Global Pose", true);
      alignPoseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    } else {
      Logger.recordOutput("Drive/Align Using Global Pose", false);
    }
  }

  @Override
  public void acceptSingleTag(Pose2d visionRobotPoseMeters, double timestampSeconds) {

    alignPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }
  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }
}
