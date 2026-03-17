// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  // Declare variable to hold a class wide solenoid refrence
  DoubleSolenoid extensionSolenoid; // Off is retracted
  SparkMax rollerMotor = new SparkMax(ActuatorConstants.kRollerCANID, MotorType.kBrushless);


  public IntakeSubsystem(PneumaticHub pneumatics) {

    // Use given pneumatic hub to define the extension solenoid
    extensionSolenoid = pneumatics.makeDoubleSolenoid(ActuatorConstants.kintakeExtensionChannel, ActuatorConstants.kintakeRetractionChannel);

    // Configure roller motor
    SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
    rollerMotorConfig
      .inverted(ActuatorConstants.kRollerInverted)
      .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit);
    rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Standard and command methods to run the intake roller
  private void setRollerPower(double powerPercentage) { rollerMotor.set(powerPercentage); }
  public Command setRollerPowerRunCommand(double powerPercentage) {
    return runEnd(() -> setRollerPower(powerPercentage), () -> { setRollerPower(0); });
  }
  public Command setRollerPowerInstantCommand(double powerPercentage) {
    return runOnce(() -> setRollerPower(powerPercentage));
  }

  // Standard and command methods to set the intake's extension
  private void setExtension(boolean extend) { extensionSolenoid.set( extend ? Value.kForward : Value.kReverse); }
  public Command setExtensionCommand(boolean extend) { return runOnce(() -> setExtension(extend)); }

  // Command to toggle the extension
  public Command toggleExtensionCommand() { return runOnce(() -> extensionSolenoid.toggle()); }


  @Override
  public void periodic() {
  }
}
