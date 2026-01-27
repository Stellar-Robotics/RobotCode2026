// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Subsystems.swerve;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  SparkMax rollerMotor = new SparkMax(0, MotorType.kBrushless);
  SparkMax extendMotor = new SparkMax(0, MotorType.kBrushless);


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    SparkMaxConfig extendMotorConfig = new SparkMaxConfig();

    rollerMotorConfig.inverted(false)
      .smartCurrentLimit(40);
    extendMotorConfig.inverted(false)
      .smartCurrentLimit(40);
    
    rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendMotor.configure(extendMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

    private void intakeFuel() {
      rollerMotor.set(0.4);
    }
   private void stopIntake() {
      rollerMotor.stopMotor();
    }


  @Override
  public void periodic() {
  }
}
