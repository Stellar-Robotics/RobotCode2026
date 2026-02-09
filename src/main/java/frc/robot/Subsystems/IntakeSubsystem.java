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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  SparkMax rollerMotor = new SparkMax(MotorConstants.kRollerCANID, MotorType.kBrushless);
  SparkMax extensionMotor = new SparkMax(MotorConstants.kintakeExtensionCANID, MotorType.kBrushless);

  SparkClosedLoopController extensionCLC = extensionMotor.getClosedLoopController();


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();


    rollerMotorConfig
      .inverted(MotorConstants.kRollerInverted)
      .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit);
    extensionMotorConfig
      .inverted(MotorConstants.kintakeExtensionInverted)
      .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit)
      .closedLoop.pid(
        MotorConstants.kintakeExtensionPID[0], 
        MotorConstants.kintakeExtensionPID[1], 
        MotorConstants.kintakeExtensionPID[2]
      );
    
    rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extensionMotor.configure(extensionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command runRollerCommand(double intakeSpeed) {

    Command intakeFuelCommand = runEnd(
      () -> {
        rollerMotor.set(intakeSpeed);
      },
      () -> {
        rollerMotor.stopMotor();
      }
    );

    // 
    return intakeFuelCommand;
  }



  // TODO: Create command to extend and retract intake
  public boolean extendMode = false;
  // true is when it is extended
  public Command toggleExtension() {
    Command setExtendMode = runOnce(() -> {
        if(extendMode == true) {
          // retract
          extensionCLC.setSetpoint(23456789, ControlType.kPosition);                    // change this
          extendMode = false;
        }
        else {
          // extend
          extensionCLC.setSetpoint(5456789, ControlType.kPosition);                     // change this too
          extendMode = true;
        }
      }
    );
    return setExtendMode;
  }
  

  double matchTime = DriverStation.getMatchTime();
  public Command initialExtend() {
    Command initialExtend = runOnce(()->{
        if(matchTime >= 23535) {                                          //change this 
          toggleExtension();
        }

      }
    );
      return initialExtend;
  }


  @Override
  public void periodic() {
  }
}
