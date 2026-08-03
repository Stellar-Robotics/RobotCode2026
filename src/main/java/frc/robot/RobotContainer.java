// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.StellarHID.CommandStellarHID;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Whether the use the custom Stellar controller or a standard Xbox controller
  boolean useCustomController = true;

  // Autonomous program selector
  SendableChooser<Command> autoSelector;
  
  // Create a class-wide accessable operator controller object
  CommandXboxController operatorController = new CommandXboxController(MiscConstants.kOperatorPort);

  // We will only use one of these, so we'll only declare them for now
  CommandStellarHID stellarDriveController;
  CommandXboxController xboxDriveController;

  // Declare subsystems, but do not define them yet
  SwerveSubsystem swerveChassis;
  IntakeSubsystem intakeSubsystem;
  HopperSubsystem hopperSubsystem;
  ShooterSubsystem shooterSubsystem;


  // This method will be called only once when the robot starts
  public RobotContainer() {

    initSwerve();

    if (MiscConstants.kUsePathplanner) {
      // Init AutoBuilder
      swerveChassis.initPathPlanner();
    }

    initMechanisms();

    if (MiscConstants.kUsePathplanner) {
      // Build list of pathplanner autos and publish them as a selector
      autoSelector = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Select Auto", autoSelector);

      // Logical barrier to keep the robot in a defined space from where it started
      swerveChassis.setLogicalBarrier();
    }
  }


  // Dedicated method to initialize the mechanisms and assign button binds
  private void initMechanisms() {

    // Shared Pneumatic hub
    PneumaticHub airBender = new PneumaticHub(ActuatorConstants.kPneumaticHubCANID);

    // Define subsystems
    intakeSubsystem = new IntakeSubsystem(airBender);
    hopperSubsystem = new HopperSubsystem();
    shooterSubsystem = new ShooterSubsystem(swerveChassis);

    
    /* -------------------------
     * Command Actions
     * ------------------------- */

    // Intake Fuel Action
    Command intakeFuel = new ParallelCommandGroup(
      intakeSubsystem.setRollerPowerRunCommand(1),
      hopperSubsystem.runHopperMechsRunCommand(false, false, false, true)
    );

    // Expel Fuel Action
    Command expelFuel = new ParallelCommandGroup(
      intakeSubsystem.setRollerPowerRunCommand(-1),
      hopperSubsystem.runHopperMechsRunCommand(true, true, true, true)
    );

    // Extend/Retract Intake Action
    Command toggleIntakeExtension = intakeSubsystem.toggleExtensionCommand();

    // Shooter Action (Spins up the shooter then feeds the fuel after a 1.5 seconds wait)
    // Command shootFuelClose = new SequentialCommandGroup(
    //   shooterSubsystem.setBonnetPositionCommand(0),
    //   shooterSubsystem.setFlywheelSpeedCommand(180),
    //   new WaitCommand(1.5),
    //   hopperSubsystem.runHopperMechsRunCommand(false, true, true, true)
    // ).handleInterrupt(() -> { 
    //   shooterSubsystem.stopShooter(); 
    //   CommandScheduler.getInstance().schedule(shooterSubsystem.setBonnetPositionCommand(0));
    // });

    // Shooter Actions (Spins up the shooter then feeds the fuel after a 1.5 seconds wait)
    Command shootFuelClose = new SequentialCommandGroup(
      // Changed from 3800 for fair
      shooterSubsystem.setShooterProfileCommand(2800, 0),
      new WaitCommand(ActuatorConstants.kFlywheelSpinUpTime),
      hopperSubsystem.runHopperMechsRunCommand(false, true, true, true)
    ).handleInterrupt(() -> shooterSubsystem.setShooterProfile(2000, 0));

    Command transportFuel = new SequentialCommandGroup(
      shooterSubsystem.setShooterProfileCommand(3800, 10),
      new WaitCommand(ActuatorConstants.kFlywheelSpinUpTime),
      hopperSubsystem.runHopperMechsRunCommand(false, true, true, true)
    ).handleInterrupt(() -> shooterSubsystem.setShooterProfile(2000, 0));

    

    // Command shootFuelFar = new SequentialCommandGroup(
    //   shooterSubsystem.setShooterProfileCommand(6000, 10),
    //   new WaitCommand(ActuatorConstants.kFlywheelSpinUpTime),
    //   hopperSubsystem.runHopperMechsRunCommand(false, true, true, true)
    // ).handleInterrupt(() -> shooterSubsystem.setShooterProfile(0, 0));

    // Command shootFuelDynamic = new ParallelCommandGroup(
    //   shooterSubsystem.autoAimRunCommand(),
    //   new SequentialCommandGroup(
    //     new WaitCommand(ActuatorConstants.kFlywheelSpinUpTime),
    //     hopperSubsystem.runHopperMechsRunCommand(false, true, true, true)
    //   )
    // );

    Command runEverythingBack = new SequentialCommandGroup(
      hopperSubsystem.runHopperMechsRunCommand(true, true, true, true)
    );


    /* -------------------------
      * Bind Commands to Triggers
      * ------------------------- */

    // Teleop Start Actions
    if (MiscConstants.kTeleopExtendIntake) {
        RobotModeTriggers.teleop().onTrue(intakeSubsystem.setExtensionCommand(false));
    }

    RobotModeTriggers.disabled().onTrue(shooterSubsystem.setShooterProfileCommand(0, 0));
    // Controller triggers
    operatorController.leftBumper().whileTrue(intakeFuel);
    operatorController.leftTrigger(0.5).whileTrue(expelFuel);
    operatorController.y().onTrue(toggleIntakeExtension);
    operatorController.povLeft().or(operatorController.povRight()).whileTrue(transportFuel);
    operatorController.povDown().whileTrue(shootFuelClose);
    operatorController.rightTrigger(0.5).whileTrue(shootFuelClose);
    operatorController.b().whileTrue(runEverythingBack);
    operatorController.x().onTrue(shooterSubsystem.setShooterProfileCommand(0, 0));
    stellarDriveController.rightBottom().onTrue(intakeSubsystem.setExtensionCommand(false));
    stellarDriveController.rightTop().onTrue(intakeSubsystem.setExtensionCommand(true));



    if (MiscConstants.kUsePathplanner) {

      // Path Finding Setup
      Pose2d shootPoseLeft = new Pose2d(3, 5, Rotation2d.fromDegrees(-37.45));
      Pose2d shootPoseRight = new Pose2d(3, 3.000, Rotation2d.fromDegrees(30));
      Pose2d shootPoseCenter = new Pose2d(2.683, 4.05, Rotation2d.fromDegrees(0));


      Pose2d outOfTheWayLeft = new Pose2d(0.8, 7.306, Rotation2d.fromDegrees(0));
      Pose2d outOfTheWayRight = new Pose2d(0.8, 2.594, Rotation2d.fromDegrees(0));

      Pose2d outOfTheWayCenter = new Pose2d(0.714, 4.960, Rotation2d.fromDegrees(0));
      Pose2d extraPoint = new Pose2d(1.723, 4.960, Rotation2d.fromDegrees(0));
      Pose2d shootPoseLeftRotated = new Pose2d(3, 5, Rotation2d.fromDegrees(0));
      Pose2d shootPoseRightRotated = new Pose2d(3, 3.000, Rotation2d.fromDegrees(0));
      Pose2d depotApproachPoint = new Pose2d(1.865, 5.077, Rotation2d.fromDegrees(-20.000));

      Pose2d outerDepot = new Pose2d(0.892, 5.964, Rotation2d.fromDegrees(0));

      PathConstraints commonPathConstraints = new PathConstraints(
        2, 2.0,
        Units.degreesToRadians(250), Units.degreesToRadians(480)
      );
      
      // Autonomous bindings (Store in a hashmap (key/val pairs))
      HashMap<String, Command> autoCommandBindings = new HashMap<>();

      // Create key/val pairs of commands we want to map (All should be instant commands)
      // Intake bindings
      autoCommandBindings.put("extendIntake", intakeSubsystem.setExtensionCommand(true));
      autoCommandBindings.put("retractIntake", intakeSubsystem.setExtensionCommand(false));
      autoCommandBindings.put("setIntakeIn", intakeSubsystem.setRollerPowerInstantCommand(1));
      autoCommandBindings.put("setIntakeOut", intakeSubsystem.setRollerPowerInstantCommand(-1));
      autoCommandBindings.put("stopIntake", intakeSubsystem.setRollerPowerInstantCommand(0));

      // Hopper Bindings
      autoCommandBindings.put("setHopperFeed", hopperSubsystem.runHopperMechsInstantCommand(false, true, true, true));
      autoCommandBindings.put("setHopperFeed3S", hopperSubsystem.runHopperMechsRunCommand(false, true, true, true)
        .withTimeout(3));
      autoCommandBindings.put("setHopperExpel", hopperSubsystem.runHopperMechsInstantCommand(true, true, true, true));
      autoCommandBindings.put("setHopperStop", hopperSubsystem.runOnce(() -> hopperSubsystem.stopAll()));

      // Shooter bindings
      autoCommandBindings.put("shooterPresetClose", shooterSubsystem.setShooterProfileCommand(3800, 0));
      autoCommandBindings.put("shooterPresetMid", shooterSubsystem.setShooterProfileCommand(ActuatorConstants.shooterPresets[1][3], ActuatorConstants.shooterPresets[1][2]));
      autoCommandBindings.put("shooterPresetfar", shooterSubsystem.setShooterProfileCommand(ActuatorConstants.shooterPresets[2][3], ActuatorConstants.shooterPresets[2][2]));
      autoCommandBindings.put("shooterPresetStop", shooterSubsystem.setShooterProfileCommand(0, 0));

      // Path Finding
      // Shooting Poses
      autoCommandBindings.put("shootPoseLeft", AutoBuilder.pathfindToPoseFlipped(shootPoseLeft, commonPathConstraints));
      autoCommandBindings.put("shootPoseCenter", AutoBuilder.pathfindToPoseFlipped(shootPoseCenter, commonPathConstraints));
      autoCommandBindings.put("shootPoseRight", AutoBuilder.pathfindToPoseFlipped(shootPoseRight, commonPathConstraints));

      // Sideline Poses
      autoCommandBindings.put("outOfTheWayLeft", AutoBuilder.pathfindToPoseFlipped(outOfTheWayLeft, commonPathConstraints));
      autoCommandBindings.put("outOfTheWayRight", AutoBuilder.pathfindToPoseFlipped(outOfTheWayRight, commonPathConstraints));
      autoCommandBindings.put("outOfTheWayCenter", AutoBuilder.pathfindToPoseFlipped(outOfTheWayCenter, commonPathConstraints));
      autoCommandBindings.put("extraPoint", AutoBuilder.pathfindToPoseFlipped(extraPoint, commonPathConstraints));
      autoCommandBindings.put("outerDepot", AutoBuilder.pathfindToPoseFlipped(outerDepot, commonPathConstraints));
      autoCommandBindings.put("shootPoseLeftRotated", AutoBuilder.pathfindToPoseFlipped(shootPoseLeftRotated, commonPathConstraints));
      autoCommandBindings.put("shootPoseRightRotated", AutoBuilder.pathfindToPoseFlipped(shootPoseRightRotated, commonPathConstraints));
      autoCommandBindings.put("depotApproachpoint", AutoBuilder.pathfindToPoseFlipped(depotApproachPoint, commonPathConstraints));
      
      // To Path then follow
      try {
        autoCommandBindings.put("sweepLeftFromTrench", AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile("SweepLeftFromTrench"), commonPathConstraints));
        autoCommandBindings.put("sweepRightFromTrench", AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile("SweepRightFromTrench"), commonPathConstraints));
        autoCommandBindings.put("depotApproach", AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile("DepotApproach"), commonPathConstraints));
      } catch (Exception e) {
        System.out.println("Could not load path from file for pathfinding! Some pathfinding autos may not work!");
      }


      // Register bindings in the HashMap
      NamedCommands.registerCommands(autoCommandBindings);
      autoCommandBindings.forEach((key, val) -> new EventTrigger(key).onTrue(val));
    }
  }


  // Dedicated method to initialize the swerve chassis and set bindings
  private void initSwerve() {

    // Define swerve chassis
     swerveChassis = new SwerveSubsystem("swerve");

    if (useCustomController) { // Create Stellar controller and setup the swerve to use it

      // Define custom controller object
      stellarDriveController = new CommandStellarHID(MiscConstants.kDriverPort);

      // Shorten our deadband constant
      double deadband = MiscConstants.kDriverDeadband;
      

      // Create and set the default drive command
      Command driveCommand = swerveChassis.driveFieldOrientedWithAbsoluteYawCommand(
        () -> -stellarDriveController.getLeftY(), 
        () -> -stellarDriveController.getLeftX(), 
        () -> stellarDriveController.getRightRotary(), 
        deadband
      ).handleInterrupt(() -> {
        swerveChassis.lock();
      });
        
      swerveChassis.setDefaultCommand(driveCommand.withName("DefaultDriveCommand"));

      // Obtain angle diff to hub and create orbit drive command, then bind it to right center
      Supplier<Pose2d> targetHub = () -> MiscUtils.isRedAlliance().getAsBoolean() ? MiscConstants.kRedHubPosition : MiscConstants.kBlueHubPosition;
      Command driveAndOrbitCommand = swerveChassis.driveFieldOrientedWithOrbit(
        () -> -stellarDriveController.getLeftY(), 
        () -> -stellarDriveController.getLeftX(), 
        () -> PhotonUtils.getYawToPose(swerveChassis.getSwerveDrive().getPose(), targetHub.get()),
        deadband
      );
      stellarDriveController.rightCenter().whileTrue(driveAndOrbitCommand);

      /*
       * Other button binds for stellar controller
       */

      // Bind the center button on the controller for zeroing the gyro
      stellarDriveController.center().onTrue(
        Commands.runOnce(() -> {
          swerveChassis.zeroGyro();
        }, swerveChassis)
      );

      // Bind experimental follow me mode to a spare button
      //stellarDriveController.leftTop().whileTrue(swerveChassis.followMe(3, 5));

    } else { // Setup Xbox controller and setup the swerve to use it

      // Define xbox controller object
      xboxDriveController = new CommandXboxController(0);

      // Create an input stream to convert between robot relative and field oriented control
      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
        swerveChassis.getSwerveDrive(),
        () -> xboxDriveController.getLeftY() * -1,
        () -> xboxDriveController.getLeftX() * -1
      )
        .withControllerRotationAxis(() -> xboxDriveController.getRightX() * -1)
        .deadband(0.2)
        .scaleTranslation(0.5)
        .allianceRelativeControl(true);

      // Create a command using the input stream to drive the robot
      Command driveFieldOrientedAngularVelocity = swerveChassis.driveFieldOrientedCommand(driveAngularVelocity);

      // Set the created command above as the default command for the swerve chassis
      swerveChassis.setDefaultCommand(driveFieldOrientedAngularVelocity);

      // Bind the back button on the controller for zeroing the gryo
      xboxDriveController.back().onTrue(
        Commands.runOnce(() -> {
          swerveChassis.zeroGyro();
        }, swerveChassis)
      );
    }
    
  }

  
  public Command getAutonomousCommand() {

    // If using pathplanner
    if (MiscConstants.kUsePathplanner) {

      // Source auto selection from pathplanner options on dashboard
      Command selectedAuto = autoSelector.getSelected();
      if (selectedAuto != null) { return selectedAuto; } // Return selected auto if it exists
      else { return new Command() {}; } // return empty command if no auto is selected

    } else {

      // Alternate auto:

      // Create and return command to drive robot forward in the x (field relative)
      // for a specific distance, and then stop.
      ChassisSpeeds desiredSpeed = new ChassisSpeeds(0.5, 0, Units.degreesToRadians(3.8));

      Command driveCommand = swerveChassis.run(
        () -> {
            swerveChassis.driveFieldOriented(desiredSpeed);
        }
      ).until(() -> swerveChassis.getOdometryEstimate().getX() >= 7.5);

      return driveCommand;
    }
  }
}
