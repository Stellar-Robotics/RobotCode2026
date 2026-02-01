// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class HopperSubsystem extends SubsystemBase {

  SparkMax beltMotor = new SparkMax(MotorConstants.kBeltCANID, MotorType.kBrushless);
  SparkMax corralMotor = new SparkMax(MotorConstants.kCorralCANID, MotorType.kBrushless);
  SparkMax kickerMotor = new SparkMax(MotorConstants.kKickerCANID, MotorType.kBrushless);

  public HopperSubsystem() {
    SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    SparkMaxConfig corralMotorConfig = new SparkMaxConfig();
    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

    beltMotorConfig.inverted(MotorConstants.kBeltInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit);
    corralMotorConfig.inverted(MotorConstants.kCorralInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit);
    kickerMotorConfig.inverted(MotorConstants.kKickerInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit)
    .closedLoop.pid(MotorConstants.kKickerPID[0], MotorConstants.kKickerPID[1], MotorConstants.kKickerPID[2]);

    kickerMotorConfig.encoder.positionConversionFactor(MotorConstants.kKickerConversionFactor);
    
    beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    corralMotor.configure(corralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runBeltCommand(double beltMotorSpeed) {
     Command runBeltCommand = runEnd(
        () -> {
          beltMotor.set(beltMotorSpeed);
        },
        () -> {
          beltMotor.stopMotor();
        }
      );
    runBeltCommand.setName("RunBelt");
    return runBeltCommand;
  }

  public Command runKickerCommand(double kickerMotorSpeed) {
     Command runKickerCommand = runEnd(
        () -> {
          kickerMotor.set(kickerMotorSpeed);
        },
        () -> {
          kickerMotor.stopMotor();
        }
      );
    runKickerCommand.setName("RunKicker");
    return runKickerCommand;
  }

  public Command runCorralCommand(double corralMotorSpeed) {
     Command runCorralCommand = runEnd(
        () -> {
          corralMotor.set(corralMotorSpeed);
        },
        () -> {
          corralMotor.stopMotor();
        }
      );

    runCorralCommand.setName("RunCorral");
    return runCorralCommand;
  }

  public Command runHopperMechs(double power, boolean corral, boolean kicker, boolean belt) {

    Command runCommand = runEnd(
      () -> {
        corralMotor.set(corral ? power : 0);
        kickerMotor.set(kicker ? power : 0);
        beltMotor.set(belt ? power : 0);
      },
      () -> {
        corralMotor.stopMotor();
        kickerMotor.stopMotor();
        beltMotor.stopMotor();
      }
    );

    runCommand.setName("RunSelectiveHopperComps");
    return runCommand;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
