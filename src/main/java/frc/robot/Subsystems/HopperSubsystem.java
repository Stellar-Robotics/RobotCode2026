// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;

public class HopperSubsystem extends SubsystemBase {

  SparkMax beltMotor = new SparkMax(ActuatorConstants.kBeltCANID, MotorType.kBrushless);
  SparkMax corralMotor = new SparkMax(ActuatorConstants.kCorralCANID, MotorType.kBrushless);
  SparkMax kickerMotor = new SparkMax(ActuatorConstants.kKickerCANID, MotorType.kBrushless);

  public HopperSubsystem() {
    SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    SparkMaxConfig corralMotorConfig = new SparkMaxConfig();
    SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

    beltMotorConfig.inverted(ActuatorConstants.kBeltInverted)
    .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit);
    corralMotorConfig.inverted(ActuatorConstants.kCorralInverted)
    .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit);
    kickerMotorConfig.inverted(ActuatorConstants.kKickerInverted)
    .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit)
    .closedLoop.pid(ActuatorConstants.kKickerPID[0], ActuatorConstants.kKickerPID[1], ActuatorConstants.kKickerPID[2]);

    kickerMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kKickerConversionFactor);
    
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

  public Command runHopperMechs(boolean reversed, boolean corral, boolean kicker, boolean belt) {

    // Get the sign(+/-) of the power input to control the direction of oscillations.
    double powerSign = Math.signum(reversed ? -1 : 1);

    // Setup and equation for a hybrid wave to oscillate the corral motor
    double frequency = 3;
    double const1 = 0.85; // Duty cycle (0 to 1)
    double  const2 = 0.05; // Smoothing effect (0 to 1)
    double const3 = Math.sqrt(Math.pow(const2, 2) + Math.pow((1 + const1), 2)) / (1 + const1); // Scaler
    Supplier<Double> offsetSinWave = () -> Math.sin(Timer.getFPGATimestamp() * frequency) + const1;
    Supplier<Double> hybridWave = () -> (offsetSinWave.get() * const3 / Math.sqrt(Math.pow(const2, 2) + Math.pow(offsetSinWave.get(), 2))) * 0.75 + 0.25;

    Command runCommand = runEnd(
      () -> {
        corralMotor.set(corral ? (hybridWave.get() * powerSign) : 0);
        kickerMotor.set(kicker ? powerSign : 0);
        // Create a sin wave that will oscilate the belts to keep fuel from jamming.
        //beltMotor.set(belt ? (Math.sin(Timer.getFPGATimestamp() * frequency) * 0.75 + (0.25 * powerSign)) : 0);
        beltMotor.set(belt ? (hybridWave.get() * powerSign) : 0);
        SmartDashboard.putNumber("HybridSquare", hybridWave.get() * powerSign);
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
