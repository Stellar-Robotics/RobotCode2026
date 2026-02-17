// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Subsystems.ClimberSubsystem;
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
  CommandXboxController operatorController = new CommandXboxController(1);

  // We will only use one of these, so we'll only declare them for now
  CommandStellarHID stellarDriveController;
  CommandXboxController xboxDriveController;

  // Declare subsystems, but do not define them yet
  SwerveSubsystem swerveChassis;

  IntakeSubsystem intakeSubsystem;
  HopperSubsystem hopperSubsystem;
  ShooterSubsystem shooterSubsystem;
  ClimberSubsystem climberSubsystem;


  // This method will be called once when the robot starts
  public RobotContainer() {

    // initMechanisms();
    initSwerve();

    if (MiscConstants.kUsePathplanner) {
      // Init AutoBuilder
      swerveChassis.initPathPlanner();

      // Build list of pathplanner autos and publish them as a selector
      autoSelector = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Select Auto", autoSelector);
    }
  }


  // Dedicated method to initialize the mechanisms and assign button binds
  private void initMechanisms() {

    // Shared Pneumatic hub
    PneumaticHub airBender = new PneumaticHub(ActuatorConstants.kPneumaticHubCANID);

    // Define subsystems
    climberSubsystem = new ClimberSubsystem(airBender);
    intakeSubsystem = new IntakeSubsystem(airBender);
    hopperSubsystem = new HopperSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    
    /* -------------------------
     * Command Actions
     * ------------------------- */

    // Intake Fuel Action
    Command intakeFuel = new ParallelCommandGroup(
      intakeSubsystem.setRollerPowerCommand(0.75),
      hopperSubsystem.runHopperMechs(0.25, false, false, true)
    );

    // Expel Fuel Action
    Command expelFuel = new ParallelCommandGroup(
      intakeSubsystem.setRollerPowerCommand(-0.75),
      hopperSubsystem.runHopperMechs(-0.5, false, true, true)
    );

    // Extend/Retract Intake Action
    Command toggleIntakeExtension = intakeSubsystem.toggleExtensionCommand();

    // Shooter Action (Spins up the shooter then feeds the fuel after a 3 seconds wait)
    Command shootFuel = new SequentialCommandGroup(
      shooterSubsystem.setFlywheelSpeed(4000),
      new WaitCommand(3),
      hopperSubsystem.runHopperMechs(0.5, true, true, true)
    ).handleInterrupt(() -> shooterSubsystem.stopShooter());

    // Extend/Retract Climber Action
    Command toggleClimberExtension = climberSubsystem.toggleExtensionCommand();

    // Endgame Climb Action
    Command climbEndgame = climberSubsystem.executeClimbSequenceCommand();

    // Teleop Start Actions
    Command teleopInit = new SequentialCommandGroup(
      climberSubsystem.disengageAutoClimb(),
      intakeSubsystem.setExtensionCommand(true)
    );


    /* -------------------------
      * Bind Commands to Triggers
      * ------------------------- */

    // Mode triggers
    RobotModeTriggers.teleop().onTrue(teleopInit);

    // Controller triggers
    operatorController.leftBumper().whileTrue(intakeFuel);
    operatorController.leftTrigger(0.5).whileTrue(expelFuel);
    operatorController.y().onTrue(toggleIntakeExtension);
    operatorController.leftTrigger(0.5).whileTrue(shootFuel);
    operatorController.back().onTrue(toggleClimberExtension);
    operatorController.start().and(operatorController.x()).onTrue(climbEndgame);
  }


  // Dedicated method to initialize the swerve chassis and set bindings
  private void initSwerve() {

    // Define swerve chassis
     swerveChassis = new SwerveSubsystem("swerve");

    if (useCustomController) { // Create Stellar controller and setup the swerve to use it

      // Define custom controller object
      stellarDriveController = new CommandStellarHID(0);

      // Shorten our deadband constant
      double deadband = MiscConstants.kDriverDeadband;
      

      // Create and set the default drive command
      Command driveCommand = swerveChassis.driveFieldOrientedWithAbsoluteYaw(
        () -> -stellarDriveController.getLeftY(), 
        () -> -stellarDriveController.getLeftX(), 
        () -> stellarDriveController.getRightRotary(), 
        deadband
      );
      swerveChassis.setDefaultCommand(driveCommand);

      // Obtain angle diff to hub and create orbit drive command, then bind it to right center
      Pose2d targetHub = MiscUtils.isRedAlliance().getAsBoolean() ? MiscConstants.kRedHubPosition : MiscConstants.kBlueHubPosition;
      Command driveAndOrbitCommand = swerveChassis.driveFieldOrientedWithOrbit(
        () -> -stellarDriveController.getLeftY(), 
        () -> -stellarDriveController.getLeftX(), 
        () -> PhotonUtils.getYawToPose(swerveChassis.getSwerveDrive().getPose(), targetHub),
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
