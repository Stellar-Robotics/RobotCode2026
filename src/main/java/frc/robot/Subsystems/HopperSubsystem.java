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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

  SparkMax beltMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax corralMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax kickerMotor = new SparkMax(0, MotorType.kBrushless);

  public HopperSubsystem() {
    SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    SparkMaxConfig corralMotorConfig = new SparkMaxConfig();
    SparkMaxConfig KickerMotorConfig = new SparkMaxConfig();

    beltMotorConfig.inverted(false)
    .smartCurrentLimit(40);
    corralMotorConfig.inverted(false)
    .smartCurrentLimit(40);
    KickerMotorConfig.inverted(false)
    .smartCurrentLimit(40);
    
    beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    corralMotor.configure(corralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(KickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
