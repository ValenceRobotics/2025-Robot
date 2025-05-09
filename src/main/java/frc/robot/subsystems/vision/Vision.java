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

  public void changePipelineIndex() {
    for (int i = 0; i < io.length; i++) {
      io[i].changePipelineIndex();
    }
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

        addSingleTagObservation(
            inputs[cameraIndex].singleTagPoseObservations,
            inputs[cameraIndex].latestTargetObservation,
            cameraIndex);
      }
      // Check if any camera has a valid tag ID

      boolean anyValidTag = false;

      for (int i = 0; i < inputs.length; i++) {

        if (inputs[i].singleTagPoseObservations.tagId() != 0) {
          anyValidTag = true;
          break;
        }
      }

      Logger.recordOutput("Vision/anyValidTag", anyValidTag);
      RobotState.setSingleTagMode(
          anyValidTag ? SingleTagMode.Available : SingleTagMode.NotAvailable);

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

    if (RobotState.getAlignState() == AlignState.InAlignRange
        && RobotState.getSingleTagMode() == SingleTagMode.Available) {
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
   * Processes and adds a single AprilTag observation to the vision system.
   *
   * <p>This method calculates the robot's position based on a single AprilTag observation using tx
   * (azimuth) and ty (elevation) angles. It performs several geometric transformations to convert
   * camera-relative measurements into field-relative robot poses.
   *
   * @param observation The SingleTagPoseObservation containing tag ID, timestamp, and distance
   *     information
   * @param target The TargetObservation containing tx and ty angles to the observed tag
   * @param cameraIndex The index of the camera that made this observation
   *     <p>The method will skip processing if: - There is already a newer observation for the same
   *     tag - The observed tag ID is not found in the AprilTag layout
   *     <p>The calculated robot pose is stored in txTyPoses map along with the distance and
   *     timestamp.
   */
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
