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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = DriveConstants.aprilTagLayout;

  // used the json file for custom apriltag removal for photonvision sim which was able to see tags
  // blocked by reef

  //   public static AprilTagFieldLayout aprilTagLayout;

  //   static {
  //     try {
  //       aprilTagLayout =
  // AprilTagFieldLayout.loadFromResource("/frc/robot/util/2025AprilTags.json");
  //       aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  //     } catch (IOException e) {
  //       e.printStackTrace();
  //     }
  //   }

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FrontLeftCam";
  public static String camera1Name = "FrontRightCam";
  public static String camera2Name = "BackCam";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(11.854),
          Units.inchesToMeters(7.019),
          Units.inchesToMeters(4.565),
          new Rotation3d(0.0, Units.degreesToRadians(-24), Units.degreesToRadians(-45)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(12.46),
          Units.inchesToMeters(-6.987),
          Units.inchesToMeters(4.565),
          new Rotation3d(0.0, Units.degreesToRadians(-24), Units.degreesToRadians(45)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(-11.9),
          Units.inchesToMeters(0),
          Units.inchesToMeters(3.125 + 2.75),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Math.PI));

  public static Transform3d[] robotToCamera = new Transform3d[] {robotToCamera0, robotToCamera1};

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
