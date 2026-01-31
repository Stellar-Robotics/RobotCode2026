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

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climberMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax extendMotor = new SparkMax(0, MotorType.kBrushless);
  Servo lockingServo = new Servo(0);

  SparkClosedLoopController climberCLC = climberMotor.getClosedLoopController();
  SparkClosedLoopController extendCLC = extendMotor.getClosedLoopController();

  public ClimberSubsystem() {
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    SparkMaxConfig extendMotorConfig = new SparkMaxConfig();

    climberMotorConfig.inverted(false)
    .smartCurrentLimit(40)
    .closedLoop.pid(0.01, 0, 0.005);

    extendMotorConfig.inverted(false)
    .smartCurrentLimit(40)
    .closedLoop.pid(0.01, 0, 0.005);

    extendMotorConfig.encoder.positionConversionFactor(0);
    climberMotorConfig.encoder.positionConversionFactor(0);

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendMotor.configure(extendMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command setClimberPositionCommand(double climberExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the climber's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedClimberExtension = MathUtil.clamp(climberExtensionMillimeters, 0, 5000); // Adjust me!

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
        lockingServo.setAngle(0); //set to unlocked posion
      }
      else{
        lockingServo.setAngle(0); //set to locked position
      }

    }
  );
    return lock;
  }

  public Command setExtendPositionCommand(double extendExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the climber's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedExtendExtension = MathUtil.clamp(extendExtensionMillimeters, 0, 5000); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command ExtendPositionCommand = runOnce(() -> {
      // Pass in our clamped value as the arguement.
      extendCLC.setSetpoint(clampedExtendExtension, ControlType.kPosition);
    });

    // Return the ClimberPositionCommand object
    return ExtendPositionCommand;
  }

  
  //true is when it is extended
  public Command setExtendMode(boolean ExtendMode) {

    Command setExtendMode = runOnce(() -> {
        if(ExtendMode == true) {
          extendCLC.setSetpoint(23456789, ControlType.kPosition); //change thius
        }
        else {
          extendCLC.setSetpoint(5456789, ControlType.kPosition); ///chhange this too
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
