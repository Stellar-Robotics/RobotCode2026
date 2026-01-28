// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // Create motor (motor controller) objects.
  SparkMax flywheelMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax bonnetMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax turretMotor = new SparkMax(0, MotorType.kBrushless);

  // Store refrences to the motors' closed loop controllers.
  SparkClosedLoopController flywheelCLC = flywheelMotor.getClosedLoopController();
  SparkClosedLoopController bonnetCLC = bonnetMotor.getClosedLoopController();
  SparkClosedLoopController turretCLC = turretMotor.getClosedLoopController();
  

  /** Creates a new Shooter. */
  public ShooterSubsystem() {

    // Create configuration objects for the motor controllers.
    SparkMaxConfig flywheelMotorConfig = new SparkMaxConfig();
    SparkMaxConfig bonnetMotorConfig = new SparkMaxConfig();
    SparkMaxConfig turretMotorConfig = new SparkMaxConfig();

    // Set configuration options by calling methods on the configuration objects.
    flywheelMotorConfig
      .inverted(false)
      .smartCurrentLimit(40)
      .closedLoop.pid(0.01, 0, 0.005);
    bonnetMotorConfig
      .inverted(false)
      .smartCurrentLimit(40)
      .closedLoop.pid(0.01, 0, 0.005);
    turretMotorConfig
      .inverted(false)
      .smartCurrentLimit(30)
      .closedLoop.pid(0.01, 0, 0.005);
    
    // (NOTE: Methods Below Require These To Be Set Correctly)
    // Set conversion factors (adjust so it corresponds with millimeters).
    bonnetMotorConfig.encoder.positionConversionFactor(0);
    // Set conversion factors (adjust so it corresponds with degrees).
    turretMotorConfig.encoder.positionConversionFactor(0);
    // Set so we get accurate conversion of RPMs at the flywheel.
    flywheelMotorConfig.encoder.positionConversionFactor(0);

    // Call the configure method on the motor objects in order to apply the config objects.
    flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bonnetMotor.configure(bonnetMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  // Create methods that return command objects so we can have the CommmandScheduler run them.

  // Method that returns a command to set the target velocity of the flywheel shaft.
  public Command setFlywheelSpeed(double flywheelSpeedRPMs) {

    // Create a command with an anonymous method that sets the closed
    // loop controller to the velocity (RPMs) specified in the parameter.
    Command flywheelCommand = run(() -> {
      flywheelCLC.setSetpoint(flywheelSpeedRPMs, ControlType.kVelocity);
    });

    // Return the flywheelCommand object.
    return flywheelCommand;
  }


  // Method that returns a command to set the target extension of the bonnet.
  public Command setBonnetPositionCommand(double bonnetExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the bonnet's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedBonnetExtension = MathUtil.clamp(bonnetExtensionMillimeters, 0, 5000); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command bonnetPositionCommand = run(() -> {
      // Pass in our clamped value as the arguement.
      bonnetCLC.setSetpoint(clampedBonnetExtension, ControlType.kPosition);
    });

    // Return the bonnetPositionCommand object
    return bonnetPositionCommand;
  }


  // Method that returns a command to set the target position of the turret motor
  public Command setTurretPositionCommand(double turretPositionDegrees) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the turret's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedPosition = MathUtil.clamp(turretPositionDegrees, 0, 5000); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command turretPositionCommand = run(() -> {
      // Pass in our clamped value as the arguement.
      turretCLC.setSetpoint(clampedPosition, ControlType.kPosition);
    });

    // Return the turretPositionCommand object.
    return turretPositionCommand;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
