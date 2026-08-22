// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;

public class IntakeSubsystem extends SubsystemBase {

  SparkFlex intakeMotor = new SparkFlex(ActuatorConstants.kIntakeMotorCANID, MotorType.kBrushless);
  SparkFlex ExtendingMotor = new SparkFlex(ActuatorConstants.kExtendingMotorCANID, MotorType.kBrushless);

  SparkClosedLoopController intakeMotorCLC = intakeMotor.getClosedLoopController();
  SparkClosedLoopController ExtendingMotorCLC = ExtendingMotor.getClosedLoopController();


  public IntakeSubsystem() {
    SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
    SparkFlexConfig ExtendingMotorConfig = new SparkFlexConfig();
    

    intakeMotorConfig
      .inverted(false)
      .smartCurrentLimit(ActuatorConstants.kvortexCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kExtendingMotorPID[0], 
      ActuatorConstants.kExtendingMotorPID[1], 
      ActuatorConstants.kExtendingMotorPID[2]);

      ExtendingMotorConfig
      .inverted(false)
      .smartCurrentLimit(ActuatorConstants.kvortexCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kIntakeMotorPID[0], 
      ActuatorConstants.kIntakeMotorPID[1], 
      ActuatorConstants.kIntakeMotorPID[2]);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public BooleanSupplier isExtended() {return () -> ExtendingMotorCLC.getSetpoint() == ActuatorConstants.RetractedPosition ? false : true;}

  public Command extendIntakeCmd() {
    Command extendCmd = runOnce(() -> {
      ExtendingMotorCLC.setSetpoint(isExtended().getAsBoolean() ? 
        ActuatorConstants.RetractedPosition :
        ActuatorConstants.ExtendedPosition, 
        ControlType.kPosition);
    }
    );
    return extendCmd;
  }

  public Command intakeCommand(Boolean isIntaking) {    //"isIntaking" checks to see if you are intaking or expeling
    Command intakeCmd = runOnce(() -> {
      intakeMotor.set(isIntaking ? 1 : -1 * ActuatorConstants.intakingSpeed);
    }
    ).handleInterrupt(() -> intakeMotor.set(0));
    return intakeCmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
