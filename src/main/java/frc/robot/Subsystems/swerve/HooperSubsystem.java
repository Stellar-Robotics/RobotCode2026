// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.swerve;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HooperSubsystem extends SubsystemBase {
  SparkMax beltMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax mecMotor = new SparkMax(0, MotorType.kBrushless);

  public HooperSubsystem() {
    SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    SparkMaxConfig mecMotorConfig = new SparkMaxConfig();

    beltMotorConfig.inverted(false)
    .smartCurrentLimit(40);
    mecMotorConfig.inverted(false)
    .smartCurrentLimit(40);
    
    beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mecMotor.configure(mecMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void startBeltMotor(double beltMotorSpeed) {
    beltMotor.set(beltMotorSpeed);
  }
  private void stopBeltMotor() {
    beltMotor.set(0);
  }

  public Command startBeltMotorCommand(double beltMotorSpeed) {
    Command startBeltMotorCommand = Commands.run(()->{
      startBeltMotor(beltMotorSpeed);
    }, this);
    return startBeltMotorCommand;
  }

  public Command stopBeltMotorCommand(){
      Command stopIntake = Commands.run(()->{
        stopBeltMotor();
      }, this);
      return stopIntake;
    }



    private void startMecMotor(double mecMotorSpeed) {
    beltMotor.set(mecMotorSpeed);
  }
  private void stopMecMotor() {
    beltMotor.set(0);
  }

  public Command startMecMotorCommand(double mecMotorSpeed) {
    Command startMecMotorCommand = Commands.run(()->{
      startBeltMotor(mecMotorSpeed);
    }, this);
    return startMecMotorCommand;
  }

  public Command stopMecMotorCommand(){
      Command stopIntake = Commands.run(()->{
        stopMecMotor();
      }, this);
      return stopIntake;
    }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
