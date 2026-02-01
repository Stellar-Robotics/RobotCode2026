// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climberMotor = new SparkMax(MotorConstants.kClimberCANID, MotorType.kBrushless);
  SparkMax extensionMotor = new SparkMax(MotorConstants.kClimberExtensionCANID, MotorType.kBrushless);
  Servo lockingServo = new Servo(MotorConstants.kLatchChannel);

  SparkClosedLoopController climberCLC = climberMotor.getClosedLoopController();
  SparkClosedLoopController extendCLC = extensionMotor.getClosedLoopController();

  public ClimberSubsystem() {
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();

    climberMotorConfig.inverted(MotorConstants.kClimberInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit)
    .closedLoop.pid(MotorConstants.kClimberPID[0], MotorConstants.kClimberPID[1], MotorConstants.kClimberPID[2]);

    extensionMotorConfig.inverted(MotorConstants.kClimberExtensionInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeo550CurrentLimit)
    .closedLoop.pid(
      MotorConstants.kClimberExtensionPID[0], 
      MotorConstants.kClimberExtensionPID[1], 
      MotorConstants.kClimberExtensionPID[2]
    );

    extensionMotorConfig.encoder.positionConversionFactor(MotorConstants.kClimberExtensionConversionFactor);
    climberMotorConfig.encoder.positionConversionFactor(MotorConstants.kClimberConversionFactor);

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extensionMotor.configure(extensionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setClimberPositionCommand(double climberExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the climber's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedClimberExtension = MathUtil.clamp(
      climberExtensionMillimeters, 
      0, 
      MotorConstants.kClimberMaxExtensionMM
    ); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command climberPositionCommand = runOnce(() -> {
      // Pass in our clamped value as the arguement.
      climberCLC.setSetpoint(clampedClimberExtension, ControlType.kPosition);
    });

    // Return the ClimberPositionCommand object
    return climberPositionCommand;
  }


  //true is unlocked
  public Command lock(boolean unlock) {
    Command lock = run(() -> {
      if(unlock == true) {
        lockingServo.setAngle(MotorConstants.kLatchUnlockPosition); //set to unlocked posion
      }
      else{
        lockingServo.setAngle(MotorConstants.kLatchLockPosition); //set to locked position
      }

    }
  );
    return lock;
  }

  public Command setExtendPositionCommand(double extendExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the climber's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedExtendExtension = MathUtil.clamp(
      extendExtensionMillimeters, 
      0, 
      MotorConstants.kClimberExtensionMaxExtensionMM
    ); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command ExtendPositionCommand = runOnce(() -> {
      // Pass in our clamped value as the arguement.
      extendCLC.setSetpoint(clampedExtendExtension, ControlType.kPosition);
    });

    // Return the ClimberPositionCommand object
    return ExtendPositionCommand;
  }

  
  // true is when it is extended
  public Command setExtendMode(boolean extendMode) {

    Command setExtendMode = runOnce(() -> {
        if(extendMode == true) {
          extendCLC.setSetpoint(23456789, ControlType.kPosition); // change this
        }
        else {
          extendCLC.setSetpoint(5456789, ControlType.kPosition); // change this too
        }
      }
    );
    return setExtendMode;
  }


  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
