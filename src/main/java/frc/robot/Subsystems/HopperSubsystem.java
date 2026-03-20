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


  public void stopAll() {

    corralMotor.stopMotor();
    beltMotor.stopMotor();
    kickerMotor.stopMotor();
  }


  public void runHopperMechs(boolean reversed, boolean corral, boolean kicker, boolean belt) {

    // Get the sign(+/-) of the power input to control the direction of oscillations.
    double powerSign = Math.signum(reversed ? -1 : 1);

    // Setup and equation for a hybrid wave (corral) to oscillate the corral motor
    double corralFrequency = 1.5;
    double corralConst1 = 0.85; // Duty cycle (0 to 1)
    double  corralConst2 = 0.05; // Smoothing effect (0 to 1)
    double corralConst3 = Math.sqrt(Math.pow(corralConst2, 2) + Math.pow((1 + corralConst1), 2)) / (1 + corralConst1); // Scaler
    Supplier<Double> corralOffsetSinWave = () -> Math.sin(Timer.getFPGATimestamp() * corralFrequency) + corralConst1;
    Supplier<Double> corralHybridWave = () -> (corralOffsetSinWave.get() * corralConst3 / Math.sqrt(Math.pow(corralConst2, 2) + Math.pow(corralOffsetSinWave.get(), 2))) * 0.75 + 0.25;

    // Setup and equation for a hybrid wave (belt) to oscillate the corral motor
    double beltFrequency = 3;
    double beltConst1 = 0.92; // Duty cycle (0 to 1)
    double  beltConst2 = 0.12; // Smoothing effect (0 to 1)
    double beltConst3 = Math.sqrt(Math.pow(beltConst2, 2) + Math.pow((1 + beltConst1), 2)) / (1 + beltConst1); // Scaler
    Supplier<Double> beltOffsetSinWave = () -> Math.sin(Timer.getFPGATimestamp() * beltFrequency) + beltConst1;
    Supplier<Double> beltHybridWave = () -> (beltOffsetSinWave.get() * beltConst3 / Math.sqrt(Math.pow(beltConst2, 2) + Math.pow(beltOffsetSinWave.get(), 2))) * 0.75 + 0.25;



    corralMotor.set(corral ? (corralHybridWave.get() * powerSign) : 0);
    kickerMotor.set(kicker ? powerSign : 0);
    beltMotor.set(belt ? (beltHybridWave.get() * powerSign) : 0);
    SmartDashboard.putNumber("CorralHybridSquare", corralHybridWave.get() * powerSign);
    SmartDashboard.putNumber("BeltHybridSquare", corralHybridWave.get() * powerSign);
  }

  public Command runHopperMechsRunCommand(boolean reversed, boolean corral, boolean kicker, boolean belt) {
    return runEnd(
      () -> runHopperMechs(reversed, corral, kicker, belt),
      () -> stopAll()
    );
  }

  public Command runHopperMechsInstantCommand(boolean reversed, boolean corral, boolean kicker, boolean belt) {
    return runOnce(() -> runHopperMechs(reversed, corral, kicker, belt));
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
