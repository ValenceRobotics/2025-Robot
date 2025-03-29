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
  public static String camera2Name = "ThirdFrontCam";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(12.086),
          Units.inchesToMeters(6.637),
          Units.inchesToMeters(9.348),
          new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(-60)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(12.467),
          Units.inchesToMeters(-6.614),
          Units.inchesToMeters(9.848),
          new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(60)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(12.608),
          Units.inchesToMeters(-6.984),
          Units.inchesToMeters(24.480),
          new Rotation3d(0.0, Units.degreesToRadians(25), 0));

  public static Transform3d[] robotToCamera =
      new Transform3d[] {robotToCamera0, robotToCamera1, robotToCamera2};

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
        1.0, // Camera 1
        2.0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
