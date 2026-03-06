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

import frc.robot.Constants.ActuatorConstants;

public class ShooterSubsystem extends SubsystemBase {

  // Create motor (motor controller) objects.
  SparkMax flywheelMotor = new SparkMax(ActuatorConstants.kFlywheelCANID, MotorType.kBrushless);
  SparkMax bonnetMotor = new SparkMax(ActuatorConstants.kBonnetCANID, MotorType.kBrushless);

  // Store refrences to the motors' closed loop controllers.
  SparkClosedLoopController flywheelCLC = flywheelMotor.getClosedLoopController();
  SparkClosedLoopController bonnetCLC = bonnetMotor.getClosedLoopController();
  

  /** Creates a new Shooter. */
  public ShooterSubsystem() {

    // Create configuration objects for the motor controllers.
    SparkMaxConfig flywheelMotorConfig = new SparkMaxConfig();
    SparkMaxConfig bonnetMotorConfig = new SparkMaxConfig();

    // Set configuration options by calling methods on the configuration objects.
    flywheelMotorConfig
      .inverted(ActuatorConstants.kFlywheelInverted)
      .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kFlywheelPID[0], ActuatorConstants.kFlywheelPID[1], ActuatorConstants.kFlywheelPID[2]);
    bonnetMotorConfig
      .inverted(ActuatorConstants.kBonnetInverted)
      .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kBonnetPID[0], ActuatorConstants.kBonnetPID[1], ActuatorConstants.kBonnetPID[2]);
    
    // (NOTE: Methods Below Require These To Be Set Correctly)
    // Set conversion factors (adjust so it corresponds with millimeters).
    //bonnetMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kBonnetConversionFactor);
    // Set so we get accurate conversion of RPMs at the flywheel.
    //flywheelMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kFlywheelConversionFactor);

    // Call the configure method on the motor objects in order to apply the config objects.
    flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bonnetMotor.configure(bonnetMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  // Method that stops the flywheel motor
  public void stopShooter() { flywheelMotor.stopMotor(); }

  // Create methods that return command objects so we can have the CommmandScheduler run them.

  // Method that returns a command to set the target velocity of the flywheel shaft.
  public Command setFlywheelSpeed(double flywheelSpeedRPMs) {

    // Clamp our specified speed to a safe range
    double clampedRPM = MathUtil.clamp(flywheelSpeedRPMs, -ActuatorConstants.kFlywheelMaxRPM, ActuatorConstants.kFlywheelMaxRPM);

    // Create a command with an anonymous method that sets the closed
    // loop controller to the velocity (RPMs) specified in the parameter.
    Command flywheelCommand = runOnce(() -> {
      flywheelCLC.setSetpoint(clampedRPM, ControlType.kVelocity);
    });

    // Set the command name and return the flywheelCommand object.
    flywheelCommand.setName("SetFlywheelTo" + clampedRPM + "RPM");
    return flywheelCommand;
  }


  // Method that returns a command to set the target extension of the bonnet.
  public Command setBonnetPositionCommand(double bonnetExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the bonnet's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedBonnetExtension = MathUtil.clamp(bonnetExtensionMillimeters, 0, ActuatorConstants.kBonnetMaxExtensionMM); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command bonnetPositionCommand = runOnce(() -> {
      // Pass in our clamped value as the arguement.
      bonnetCLC.setSetpoint(clampedBonnetExtension, ControlType.kPosition);
    });

    // Set the command name and return the bonnetPositionCommand object
    bonnetPositionCommand.setName("SetBonnetTo" + clampedBonnetExtension + "MM");
    return bonnetPositionCommand;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
