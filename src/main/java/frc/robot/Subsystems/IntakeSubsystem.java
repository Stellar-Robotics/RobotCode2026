// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ActuatorConstants;

public class IntakeSubsystem extends SubsystemBase {

  SparkMax intakeMotor = new SparkMax(ActuatorConstants.kIntakeMotorCANID, MotorType.kBrushless);
  SparkMax extendingMotor = new SparkMax(ActuatorConstants.kExtendingMotorCANID, MotorType.kBrushless);

  SparkClosedLoopController intakeMotorCLC = intakeMotor.getClosedLoopController();
  SparkClosedLoopController extendingMotorCLC = extendingMotor.getClosedLoopController();


  public IntakeSubsystem() {
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    SparkMaxConfig extendingMotorConfig = new SparkMaxConfig();
    

    intakeMotorConfig
      .inverted(false)
      .smartCurrentLimit(ActuatorConstants.kvortexCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kExtendingMotorPID[0], 
      ActuatorConstants.kExtendingMotorPID[1], 
      ActuatorConstants.kExtendingMotorPID[2]);

    extendingMotorConfig
      .inverted(false)
      .smartCurrentLimit(ActuatorConstants.kvortexCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kIntakeMotorPID[0], 
      ActuatorConstants.kIntakeMotorPID[1],
      ActuatorConstants.kIntakeMotorPID[2])
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);;


    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extendingMotor.configure(extendingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public BooleanSupplier isExtended() {return () -> extendingMotorCLC.getSetpoint() == ActuatorConstants.retractedPosition ? false : true;}

  public Command toggleExtension() {//this should flip the extension to what it isnt
    Command extendCmd = runOnce(() -> {
      extendingMotorCLC.setSetpoint(isExtended().getAsBoolean() ? 
        ActuatorConstants.retractedPosition :
        ActuatorConstants.extendedPosition, 
        ControlType.kPosition);
    }
    );
    return extendCmd;
  }

  public Command extensionCommand(boolean isExtending) {//this takes in an agrument to decide if it extends or retracts
    Command extendCmd = runOnce(() -> {
      extendingMotorCLC.setSetpoint(isExtending ? 
        ActuatorConstants.extendedPosition :
        ActuatorConstants.retractedPosition, 
        ControlType.kPosition);
    }
    );
    return extendCmd;
  }

  public void oscilateExtendingMotor() {
    double frequency = 3;
    double constant = 0.85;
    double height = 7;  //this increases the amplitude(not sure about this terminology) of the wave
    /*oscilation should be height * 2 */
    double setpoint = Math.sin(Timer.getFPGATimestamp() * frequency) * height + constant;
    extendingMotorCLC.setSetpoint(setpoint, ControlType.kPosition);  /*I beleive this should oscelate 14 degrees*/
  }

  public Command intakeCommand(Boolean isIntaking) {    //"isIntaking" checks to see if you are intaking or expeling
    Command intakeCmd = runEnd(() -> {
      intakeMotor.set(isIntaking ? ActuatorConstants.intakingSpeed : -1 * ActuatorConstants.intakingSpeed);
      new WaitCommand(1);
      oscilateExtendingMotor();
    }, () -> {
      intakeMotor.set(0);
    }
    );
    return intakeCmd;
  }

  public Command stopIntake() {return runOnce(() -> intakeMotor.set(0));}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
