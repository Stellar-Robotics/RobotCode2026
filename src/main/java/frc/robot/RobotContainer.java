// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.swerve.HooperSubsystem;
import frc.robot.Subsystems.swerve.IntakeSubsystem;

public class RobotContainer {
  
  CommandXboxController operaController = new CommandXboxController(2);

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  HooperSubsystem hooperSubsystem = new HooperSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
    operaController.a().whileTrue(intakeSubsystem.intakeFuelCommand(0.4));
    operaController.b().whileTrue(intakeSubsystem.intakeFuelCommand(-0.4));
    operaController.x().whileTrue(
      new ParallelCommandGroup(
        hooperSubsystem.startMecMotorCommand(0.4),
        hooperSubsystem.startBeltMotorCommand(0.4)
      )
    );

    operaController.y().whileTrue(
      new ParallelCommandGroup(
        hooperSubsystem.startBeltMotorCommand(-0.4),
        hooperSubsystem.startMecMotorCommand(-0.4)
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
