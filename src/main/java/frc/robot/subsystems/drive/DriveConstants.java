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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 5.02;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(28);
  public static final double wheelBase = Units.inchesToMeters(28);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final double maxAngularSpeed = maxSpeedMetersPerSec / driveBaseRadius;
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final double bumperThickness = Units.inchesToMeters(6);
  public static final double endEffectorOffset = Units.inchesToMeters(2);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(Math.PI / 2);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-Math.PI / 2);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI);

  // Device CAN IDs
  public static final int pigeonCanId = 5;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 44;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.689);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (13.0 * 15.0); // MAXSwerve with 13 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.137458;
  public static final double driveKv = 0.083178;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 52.161;
  public static final double robotMOI = 5.777;
  public static final double wheelCOF = 1.5;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withBumperSize(Meters.of(0.89535), Meters.of(0.89535))
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public enum ReefTags {
    SIDE_AB(null),
    SIDE_CD(null),
    SIDE_EF(null),
    SIDE_GH(null),
    SIDE_IJ(null),
    SIDE_KL(null);

    private Optional<Pose3d> pose;

    ReefTags(Optional<Pose3d> pose) {
      this.pose = pose;
    }

    public static void initializeAlliance(BooleanSupplier isBlue) {
      if (isBlue.getAsBoolean()) {
        SIDE_AB.pose = aprilTagLayout.getTagPose(18);
        SIDE_CD.pose = aprilTagLayout.getTagPose(17);
        SIDE_EF.pose = aprilTagLayout.getTagPose(22);
        SIDE_GH.pose = aprilTagLayout.getTagPose(21);
        SIDE_IJ.pose = aprilTagLayout.getTagPose(20);
        SIDE_KL.pose = aprilTagLayout.getTagPose(19);
      } else {
        SIDE_AB.pose = aprilTagLayout.getTagPose(7);
        SIDE_CD.pose = aprilTagLayout.getTagPose(8);
        SIDE_EF.pose = aprilTagLayout.getTagPose(9);
        SIDE_GH.pose = aprilTagLayout.getTagPose(10);
        SIDE_IJ.pose = aprilTagLayout.getTagPose(11);
        SIDE_KL.pose = aprilTagLayout.getTagPose(6);
      }
    }

    public Optional<Pose3d> getPose() {
      return pose;
    }
  }

  public enum BlueCoralScoreLocation {
    A(
        aprilTagLayout
            .getTagPose(18)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) - endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    B(
        aprilTagLayout
            .getTagPose(18)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + 0,
                    new Rotation2d(Math.PI)))),
    C(
        aprilTagLayout
            .getTagPose(17)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) - endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    D(
        aprilTagLayout
            .getTagPose(17)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + 0,
                    new Rotation2d(Math.PI)))),
    E(
        aprilTagLayout
            .getTagPose(22)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) - endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    F(
        aprilTagLayout
            .getTagPose(22)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + 0,
                    new Rotation2d(Math.PI)))),
    G(
        aprilTagLayout
            .getTagPose(21)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) - endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    H(
        aprilTagLayout
            .getTagPose(21)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + 0,
                    new Rotation2d(Math.PI)))),
    I(
        aprilTagLayout
            .getTagPose(20)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) - endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    J(
        aprilTagLayout
            .getTagPose(20)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + 0,
                    new Rotation2d(Math.PI)))),
    K(
        aprilTagLayout
            .getTagPose(19)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) - endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    L(
        aprilTagLayout
            .getTagPose(19)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + 0,
                    new Rotation2d(Math.PI))));

    private final Pose2d pose;

    BlueCoralScoreLocation(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  public enum RedCoralScoreLocation {
    A(
        aprilTagLayout
            .getTagPose(7)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    B(
        aprilTagLayout
            .getTagPose(7)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    C(
        aprilTagLayout
            .getTagPose(8)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    D(
        aprilTagLayout
            .getTagPose(8)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    E(
        aprilTagLayout
            .getTagPose(9)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    F(
        aprilTagLayout
            .getTagPose(9)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    G(
        aprilTagLayout
            .getTagPose(10)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    H(
        aprilTagLayout
            .getTagPose(10)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    I(
        aprilTagLayout
            .getTagPose(11)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    J(
        aprilTagLayout
            .getTagPose(11)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    K(
        aprilTagLayout
            .getTagPose(6)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    -Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI)))),
    L(
        aprilTagLayout
            .getTagPose(6)
            .get()
            .toPose2d()
            .plus(
                new Transform2d(
                    trackWidth / 2 + bumperThickness,
                    Units.inchesToMeters(13 / 2) + endEffectorOffset,
                    new Rotation2d(Math.PI))));

    private final Pose2d pose;

    RedCoralScoreLocation(Pose2d pose) {
      this.pose = pose;
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  public enum CoralScoreLocation {
    A(null),
    B(null),
    C(null),
    D(null),
    E(null),
    F(null),
    G(null),
    H(null),
    I(null),
    J(null),
    K(null),
    L(null);

    private Pose2d pose;

    CoralScoreLocation(Pose2d pose) {
      this.pose = pose;
    }

    public static void initializeAlliance(BooleanSupplier isBlue) {
      if (isBlue.getAsBoolean()) {
        for (CoralScoreLocation location : values()) {
          location.pose = BlueCoralScoreLocation.valueOf(location.name()).getPose();
        }
      } else {
        for (CoralScoreLocation location : values()) {
          location.pose = RedCoralScoreLocation.valueOf(location.name()).getPose();
        }
      }
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  public enum HPTags {
    LEFT_STATION(null),
    RIGHT_STATION(null);

    private Pose2d pose;

    HPTags(Pose2d pose) {
      this.pose = pose;
    }

    public static void initializeAlliance(BooleanSupplier isBlue) {
      if (isBlue.getAsBoolean()) {
        LEFT_STATION.pose = aprilTagLayout.getTagPose(13).get().toPose2d();
        RIGHT_STATION.pose = aprilTagLayout.getTagPose(12).get().toPose2d();
      } else {
        LEFT_STATION.pose = aprilTagLayout.getTagPose(1).get().toPose2d();
        RIGHT_STATION.pose = aprilTagLayout.getTagPose(2).get().toPose2d();
      }
    }

    public Pose2d getPose() {
      return pose;
    }
  }
}
