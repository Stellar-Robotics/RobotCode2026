// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.HopperSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  
  CommandXboxController operatorController = new CommandXboxController(2);

  // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // HopperSubsystem hopperSubsystem = new HopperSubsystem();
  // ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  SwerveSubsystem swerveChassis = new SwerveSubsystem("swerve");

  public RobotContainer() {
    //configureBindings();
    initSwerve();
  }

  // private void configureBindings() {

  //   climberSubsystem.setDefaultCommand(climberSubsystem.setExtendMode(false));
    
  //   operatorController.a().whileTrue(intakeSubsystem.runRollerCommand(0.4));
  //   operatorController.b().whileTrue(intakeSubsystem.runRollerCommand(-0.4));
  //   operatorController.x().whileTrue(hopperSubsystem.runHopperMechs(0.5, true, true, true));


  //   // Spins up the shooter and then feeds the fuel after a 3 second delay.
  //   operatorController.leftTrigger(0.5).whileTrue(
  //     new SequentialCommandGroup(
  //       shooterSubsystem.setFlywheelSpeed(4000),
  //       new WaitCommand(3),
  //       hopperSubsystem.runHopperMechs(0.5, true, true, true)
  //     )
  //   ).onFalse(shooterSubsystem.setFlywheelSpeed(0));

  //   operatorController.back().onTrue(
  //     climberSubsystem.setExtendMode(true)
  //   );
  // }

  private void initSwerve() {

    CommandXboxController driverController = new CommandXboxController(0);

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      swerveChassis.getSwerveDrive(),
      () -> driverController.getLeftY() * -1,
      () -> driverController.getLeftX() * -1
    )
      .withControllerRotationAxis(() -> driverController.getRightX() * -1)
      .deadband(0.2)
      .scaleTranslation(1)
      .allianceRelativeControl(true);

    Command driveFieldOrientedAngularVelocity = swerveChassis.driveFieldOriented(driveAngularVelocity);
    swerveChassis.setDefaultCommand(driveFieldOrientedAngularVelocity);

    driverController.back().onTrue(
      Commands.runOnce(() -> {
        swerveChassis.zeroGyro();
      }, swerveChassis)
    );
    
  }

  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
