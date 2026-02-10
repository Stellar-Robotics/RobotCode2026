// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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



  public RobotContainer() {
    // initMechanisms();
    initSwerve();
  }


  private void initMechanisms() {

    // Define subsystems
    intakeSubsystem = new IntakeSubsystem();
    hopperSubsystem = new HopperSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    climberSubsystem = new ClimberSubsystem();

    climberSubsystem.setDefaultCommand(climberSubsystem.lock(false).ignoringDisable(true));

    climberSubsystem.setDefaultCommand(climberSubsystem.toggleExtension());
    
    operatorController.a().whileTrue(intakeSubsystem.runRollerCommand(0.4));
    operatorController.b().whileTrue(intakeSubsystem.runRollerCommand(-0.4));
    operatorController.x().whileTrue(hopperSubsystem.runHopperMechs(0.5, true, true, true));


    // Spins up the shooter and then feeds the fuel after a 3 second delay.
    operatorController.leftTrigger(0.5).whileTrue(
      new SequentialCommandGroup(
        shooterSubsystem.setFlywheelSpeed(4000),
        new WaitCommand(3),
        hopperSubsystem.runHopperMechs(0.5, true, true, true)
      )
    ).onFalse(shooterSubsystem.setFlywheelSpeed(0));

    operatorController.back().onTrue(
      climberSubsystem.toggleExtension()
    );

    operatorController.start().onTrue(
      climberSubsystem.climbing()
    );

    operatorController.y().onTrue(             //change button
      intakeSubsystem.toggleExtension()
    );
  }


    


  private void initSwerve() {

    // Define swerve chassis
     swerveChassis = new SwerveSubsystem("swerve");

    if (useCustomController) { // Create Stellar controller and setup the swerve to use it

      // Define custom controller object
      stellarDriveController = new CommandStellarHID(0);
      

      // Create and set the default drive command
      Command driveCommand = swerveChassis.driveFieldOrientedWithAbsoluteYaw(
        () -> -stellarDriveController.getLeftY(), 
        () -> -stellarDriveController.getLeftX(), 
        () -> stellarDriveController.getRightRotary(), 
        0.1
      );
      swerveChassis.setDefaultCommand(driveCommand);

      // Obtain angle diff to hub and create orbit drive command, then bind it to right center
      Pose2d targetHub = MiscUtils.isRedAlliance().getAsBoolean() ? MiscConstants.kRedHubPosition : MiscConstants.kBlueHubPosition;
      Command driveAndOrbitCommand = swerveChassis.driveFieldOrientedWithOrbit(
        () -> -stellarDriveController.getLeftY(), 
        () -> -stellarDriveController.getLeftX(), 
        () -> PhotonUtils.getYawToPose(swerveChassis.getSwerveDrive().getPose(), targetHub), // Change my yaw to point wherever the hub is!
        0.1
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
      Command driveFieldOrientedAngularVelocity = swerveChassis.driveFieldOriented(driveAngularVelocity);

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


  public void periodic() {
    SmartDashboard.putNumber("AngleDiff", PhotonUtils.getYawToPose(swerveChassis.getOdometryEstimate(), MiscConstants.kRedHubPosition).getDegrees());
  }

  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
