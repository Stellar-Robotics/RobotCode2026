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

import edu.wpi.first.wpilibj2.command.Command;
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
      .smartCurrentLimit(40)
      .closedLoop.pid(0.01, 0, 0.005);
    
    rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendMotor.configure(extendMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  private void goToPosition(int rotations){

    SparkClosedLoopController clc = extendMotor.getClosedLoopController();
    clc.setSetpoint(rotations, ControlType.kPosition);
  }


  public Command intakeFuelCommand(double intakeSpeed) {

    Command intakeFuelCommand = runEnd(
      () -> {
        rollerMotor.set(intakeSpeed);
      },
      () -> {
        rollerMotor.stopMotor();
      }
    );

    return intakeFuelCommand;
  }

  @Override
  public void periodic() {
  }
}
