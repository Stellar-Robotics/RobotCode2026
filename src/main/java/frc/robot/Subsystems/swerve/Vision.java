// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class Vision extends SubsystemBase {

  // Suppliers from the swerve subsystem
  Supplier<Pose2d> robotPose;
  Field2d field;

  public Vision(Supplier<Pose2d> poseSupplier, Field2d fieldLayout) {
    this.robotPose = poseSupplier;
    this.field = fieldLayout;
  }

  public void updateEstimatedPose(SwerveDrive swerveSubsystem) {
    System.out.println("I don't do anything yet!");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
