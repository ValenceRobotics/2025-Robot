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

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState.AimbotMode;
import frc.robot.RobotState.CoralState;
import frc.robot.RobotState.DriveState;
import frc.robot.RobotState.ElevatorState;
import frc.robot.RobotState.EndEffectorState;
import frc.robot.RobotState.SystemMode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StateCommands;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  // private final Algae algae;
  public final Elevator elevator;
  private final EndEffector endEffector;
  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0, IdleMode.kBrake),
                new ModuleIOSpark(1, IdleMode.kBrake),
                new ModuleIOSpark(2, IdleMode.kBrake),
                new ModuleIOSpark(3, IdleMode.kBrake),
                (pose) -> {});

        this.vision =
            new Vision(
                drive,
                drive::getPose,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2));
        this.elevator = new Elevator(new ElevatorIOReal());
        this.endEffector = new EndEffector(new EndEffectorIOReal());
        //  this.algae = new Algae(new AlgaeIOReal());
        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig,
                new Pose2d(3, 3, new Rotation2d(Units.degreesToRadians(0))));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    camera0Name,
                    VisionConstants.robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    VisionConstants.robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name,
                    VisionConstants.robotToCamera2,
                    driveSimulation::getSimulatedDriveTrainPose));
        elevator = new Elevator(new ElevatorIOSim());
        endEffector = new EndEffector(new EndEffectorIO() {});
        //  algae = new Algae(new AlgaeIO() {});
        break;
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, drive::getPose, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        //  algae = new Algae(new AlgaeIO() {});
        break;
    }
    // named commands declaration
    NamedCommands.registerCommand(
        "autoAlignLeft",
        DriveCommands.alignToReef(
                drive,
                () -> drive.getScoreLocations()[0],
                () -> drive.getReefPose(drive.getScoreLocations()[0]))
            .until(() -> RobotState.getDriveState() == DriveState.Aligned)
            .withTimeout(3));
    NamedCommands.registerCommand(
        "autoAlignRight",
        DriveCommands.alignToReef(
                drive,
                () -> drive.getScoreLocations()[1],
                () -> drive.getReefPose(drive.getScoreLocations()[1]))
            .until(() -> RobotState.getDriveState() == DriveState.Aligned)
            .withTimeout(3));
    NamedCommands.registerCommand(
        "autoScore1",
        (StateCommands.setMechanismState(ElevatorState.L1))
            .andThen(new WaitCommand(1))
            .andThen(StateCommands.setMechanismState(EndEffectorState.Score))
            .andThen(new WaitCommand(1))
            .andThen(StateCommands.setMechanismState(EndEffectorState.Stopped))
            .andThen(StateCommands.setMechanismState(ElevatorState.Home))
            .withTimeout(1));
    NamedCommands.registerCommand(
        "autoScore4",
        (StateCommands.setMechanismState(ElevatorState.L4Force))
            .andThen(
                new WaitCommand(1.4)
                    .andThen(StateCommands.setMechanismState(EndEffectorState.Score)))
            .andThen(new WaitUntilCommand(() -> RobotState.getCoralState() == CoralState.NoCoral))
            .andThen(StateCommands.setMechanismState(EndEffectorState.Stopped))
            .withTimeout(1.6));
    NamedCommands.registerCommand(
        "retract4", (StateCommands.setMechanismState(ElevatorState.Home)).withTimeout(.5));
    NamedCommands.registerCommand(
        "getCoralHp",
        (StateCommands.setMechanismState(EndEffectorState.Intake))
            .alongWith(StateCommands.setMechanismState(ElevatorState.Intake))
            .andThen(
                new WaitUntilCommand(() -> RobotState.getCoralState() == CoralState.HasCoral)));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // algae.setDefaultCommand(
    //     Commands.run(() -> algae.setPivotVoltage(-12 * operatorController.getLeftY()), algae));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // operatorController
    //     .leftTrigger()
    //     .whileTrue(Commands.run(() -> algae.setShooterVoltage(3)))
    //     .onFalse(Commands.run(() -> algae.setShooterVoltage(0)));

    // operatorController
    //     .rightTrigger()
    //     .whileTrue(Commands.run(() -> algae.setShooterVoltage(-3)))
    //     .onFalse(Commands.run(() -> algae.setShooterVoltage(0)));

    // temporary elevator commands
    controller.a().onTrue(StateCommands.setMechanismState(ElevatorState.L1));
    controller.b().onTrue(StateCommands.setMechanismState(ElevatorState.L2));
    controller.x().onTrue(StateCommands.setMechanismState(ElevatorState.L3));
    controller.y().onTrue(StateCommands.setMechanismState(ElevatorState.L4));
    // controller.povUp().onTrue(StateCommands.setMechanismState(ElevatorState.testing));

    controller.leftBumper().onTrue(StateCommands.setMechanismState(ElevatorState.Home));

    controller.rightBumper().onTrue(Commands.runOnce(() -> vision.changePipelineIndex()));

    // reef alignment
    controller
        .povLeft()
        .or(controller.pov(315))
        .or(controller.pov(225))
        .whileTrue(
            DriveCommands.alignToReef(
                drive,
                () -> drive.getScoreLocations()[0],
                () -> drive.getReefPose(drive.getScoreLocations()[0])));
    controller
        .povRight()
        .or(controller.pov(45))
        .or(controller.pov(135))
        .whileTrue(
            DriveCommands.alignToReef(
                drive,
                () -> drive.getScoreLocations()[1],
                () -> drive.getReefPose(drive.getScoreLocations()[1])));
    controller
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(
                () -> endEffector.setEndEffectorState(EndEffectorState.Score), endEffector))
        .onFalse(
            Commands.runOnce(
                () -> endEffector.setEndEffectorState(EndEffectorState.Stopped), endEffector));
    controller
        .rightTrigger()
        .whileTrue(
            StateCommands.setMechanismState(EndEffectorState.Intake)
                .alongWith(StateCommands.setMechanismState(ElevatorState.Intake)))
        .onFalse(
            StateCommands.setMechanismState(EndEffectorState.Stopped)
                .alongWith(StateCommands.setMechanismState(ElevatorState.Home)));

    controller
        .button(10)
        .whileTrue(StateCommands.setMechanismState(EndEffectorState.Reverse))
        .onFalse(StateCommands.setMechanismState(EndEffectorState.Stopped));

    // controller
    //     .rightBumper()
    //     .whileTrue(
    //         new DriveToPose(drive, () -> drive.getBargeShotPose())
    //             .andThen(StateCommands.setMechanismState(ElevatorState.L4Force))
    //             .andThen(new WaitUntilCommand(() -> elevator.aboveBargeRelease()))
    //             .andThen(StateCommands.setMechanismState(AlgaeState.Shoot)))
    //     .onFalse(
    //         StateCommands.setMechanismState(ElevatorState.Home)
    //             .alongWith(StateCommands.setMechanismState(AlgaeState.Stow)));

    controller
        .button(7)
        .onTrue(
            Commands.either(
                StateCommands.setMechanismState(SystemMode.Auto),
                StateCommands.setMechanismState(SystemMode.Manual),
                () -> RobotState.getSystemMode() == SystemMode.Manual));

    controller
        .button(9)
        .onTrue(
            Commands.either(
                StateCommands.setMechanismState(AimbotMode.Aimbot),
                StateCommands.setMechanismState(AimbotMode.NoAimbot),
                () -> RobotState.getAimbotMode() == AimbotMode.NoAimbot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));

    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
