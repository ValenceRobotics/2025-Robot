// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.AlignState;
import frc.robot.RobotState.SingleTagMode;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.SingleTagPoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> poseSupplier;

  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));

  private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();

  public Vision(VisionConsumer consumer, Supplier<Pose2d> poseSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.poseSupplier = poseSupplier;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      txTyPoses.put(i, new TxTyPoseRecord(Pose2d.kZero, Double.POSITIVE_INFINITY, -1.0));
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Add single tag pose observations
      if (inputs[cameraIndex].singleTagPoseObservations.tagId() != 0) {
        RobotState.setSingleTagMode(SingleTagMode.Available);
        addSingleTagObservation(
            inputs[cameraIndex].singleTagPoseObservations,
            inputs[cameraIndex].latestTargetObservation,
            cameraIndex);
      } else {
        RobotState.setSingleTagMode(SingleTagMode.NotAvailable);
      }
      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    if (RobotState.getSingleTagMode() == SingleTagMode.Available
        && RobotState.getAlignState()
            == AlignState.Aligning) { // switch to use singletag state thing
      if (getTxTyPose(RobotState.getCurrentTag().getId()).isPresent()) {
        TxTyPoseRecord record = getTxTyPose(RobotState.getCurrentTag().getId()).get();
        consumer.acceptSingleTag(record.pose(), record.timestamp());
      }
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    Logger.recordOutput("Vision/SingleTagMap", txTyPoses.toString());

    // Log single tag pose
    Logger.recordOutput("Vision/Single Tag Pose 1", getSingleTagPose(0));
    Logger.recordOutput("Vision/Single Tag Pose 2", getSingleTagPose(1));
  }

  // @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);

    void acceptSingleTag(Pose2d visionRobotPoseMeters, double timestampSeconds);
  }

  /**
   * Calculates the robot's pose based on a single AprilTag observation from a specific camera.
   *
   * @param cameraIndex The index of the camera to get pose data from
   * @return A Pose2d representing the robot's position and rotation in field coordinates. Returns
   *     an empty pose (0,0,0) if the observed tag is not in the field layout.
   *     <p>The method: 1. Gets the 3D distance to the tag and camera angles 2. Converts the 3D
   *     measurements to 2D field coordinates 3. Calculates the rotation between camera and tag 4.
   *     Uses the tag's known field position to determine robot position 5. Applies camera-to-robot
   *     transform to get final robot pose
   */
  public Pose2d getSingleTagPose(int cameraIndex) {
    SingleTagPoseObservation observation = inputs[cameraIndex].singleTagPoseObservations;
    TargetObservation target = inputs[cameraIndex].latestTargetObservation;

    double tx = target.tx().getRadians();
    double ty = target.ty().getRadians();
    Transform3d cameraPose = robotToCamera[cameraIndex];

    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, -ty, -tx))
            .transformBy(
                new Transform3d(
                    new Translation3d(observation.tagDistance(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();

    Rotation2d camToTagRotation =
        poseSupplier
            .get()
            .getRotation()
            .plus(
                Rotation2d.fromRadians(cameraPose.getRotation().getZ())
                    .plus(camToTagTranslation.getAngle()));

    var optionalTagPose = VisionConstants.aprilTagLayout.getTagPose(observation.tagId());
    if (optionalTagPose.isEmpty()) {
      return new Pose2d(); // or handle the case appropriately
    }
    Pose2d tagPose2d = optionalTagPose.get().toPose2d();

    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
            .getTranslation();

    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                poseSupplier
                    .get()
                    .getRotation()
                    .plus(Rotation2d.fromRadians(cameraPose.getRotation().getZ())))
            .transformBy(
                new Transform2d(
                    new Pose3d(cameraPose.getTranslation(), cameraPose.getRotation()).toPose2d(),
                    Pose2d.kZero));

    robotPose = new Pose2d(robotPose.getTranslation(), poseSupplier.get().getRotation());

    return robotPose;
  }

  /**
   * Calculates a blended pose between the robot's estimated pose and a single tag based pose.
   *
   * @param camIndex The index of the camera to get the tag pose from
   * @param finalPose The final destination pose used to calculate blending factor
   * @return A blended Pose2d between estimated and tag-based poses based on distance to final pose:
   *     - Returns estimated pose if no tag is visible - Interpolates between tag pose and estimated
   *     pose based on distance: - More weight on tag pose when closer to minDistanceTagPoseBlend -
   *     More weight on estimated pose when closer to maxDistanceTagPoseBlend
   */
  public Pose2d getReefPose(int camIndex, Pose2d finalPose) {
    var tagPose = getSingleTagPose(camIndex);
    // Use estimated pose if tag pose is not present
    if (tagPose.equals(new Pose2d())) {
      RobotState.setSingleTagMode(SingleTagMode.NotAvailable);
      return poseSupplier.get();
    }

    RobotState.setSingleTagMode(SingleTagMode.Available);
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (poseSupplier.get().getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);
    return poseSupplier.get().interpolate(tagPose, 1.0 - t);
  }

  public void addSingleTagObservation(
      SingleTagPoseObservation observation, TargetObservation target, int cameraIndex) {
    // Skip if current data for tag is newer
    if (txTyPoses.containsKey(observation.tagId())
        && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) {
      return;
    }

    double tx = target.tx().getRadians();
    double ty = target.ty().getRadians();
    Transform3d cameraPose = robotToCamera[cameraIndex];

    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, -ty, -tx))
            .transformBy(
                new Transform3d(
                    new Translation3d(observation.tagDistance(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();

    Rotation2d camToTagRotation =
        poseSupplier
            .get()
            .getRotation()
            .plus(
                Rotation2d.fromRadians(cameraPose.getRotation().getZ())
                    .plus(camToTagTranslation.getAngle()));

    var optionalTagPose = VisionConstants.aprilTagLayout.getTagPose(observation.tagId());
    if (optionalTagPose.isEmpty()) {
      return; // or handle the case appropriately
    }
    Pose2d tagPose2d = optionalTagPose.get().toPose2d();

    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
            .getTranslation();

    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                poseSupplier
                    .get()
                    .getRotation()
                    .plus(Rotation2d.fromRadians(cameraPose.getRotation().getZ())))
            .transformBy(
                new Transform2d(
                    new Pose3d(cameraPose.getTranslation(), cameraPose.getRotation()).toPose2d(),
                    Pose2d.kZero));

    robotPose = new Pose2d(robotPose.getTranslation(), poseSupplier.get().getRotation());

    txTyPoses.put(
        observation.tagId(),
        new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
  }

  /** Get 2d pose estimate of robot if not stale. */
  public Optional<TxTyPoseRecord> getTxTyPose(int tagId) {
    if (!txTyPoses.containsKey(tagId)) {
      return Optional.empty();
    }
    var data = txTyPoses.get(tagId);
    // Check if stale
    if (Timer.getTimestamp() - data.timestamp() >= 0.5) {
      return Optional.empty();
    }
    return Optional.of(data);
  }

  public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}
}
