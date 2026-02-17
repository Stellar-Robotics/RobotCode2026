// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.MotorConstants;

public class ClimberSubsystem extends SubsystemBase {

  // Declare holders for pneumatic hub and solenoids
  Solenoid extensionSolenoid; // Off is retracted
  Solenoid lockSolenoid; // Off is locked

  // Create motor and CLC objects
  SparkMax climberMotor = new SparkMax(MotorConstants.kClimberCANID, MotorType.kBrushless);
  SparkClosedLoopController climberCLC = climberMotor.getClosedLoopController();

  // Define known climber extensions
  int homePosition = 0; // Set me! Position the upper hooks will reset to.
  int latchPosition = 0; // Set me! Full travel to engage ladder hooks.
  int lockPosition = 0; // Set me! Somewhere between home and the latch positions (for 3rd stage only).

  // Climber encoder position supplier
  Supplier<Double> climberMotorPosition = () -> climberMotor.getEncoder().getPosition();

  // Create over/under bias to account for PID innacuracies in the climber motor
  int climberPositionBias = 5; // Tune me


  public ClimberSubsystem(PneumaticHub pneumatics) {

    // Setup pneumatics
    extensionSolenoid = pneumatics.makeSolenoid(0);
    lockSolenoid = pneumatics.makeSolenoid(1);

    // Create and configure motor configs
    SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();

    climberMotorConfig.inverted(MotorConstants.kClimberInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeoCurrentLimit)
    .closedLoop.pid(MotorConstants.kClimberPID[0], MotorConstants.kClimberPID[1], MotorConstants.kClimberPID[2]);

    extensionMotorConfig.inverted(MotorConstants.kClimberExtensionInverted)
    .smartCurrentLimit(MotorConstants.kCommonNeo550CurrentLimit)
    .closedLoop.pid(
      MotorConstants.kClimberExtensionPID[0], 
      MotorConstants.kClimberExtensionPID[1], 
      MotorConstants.kClimberExtensionPID[2]
    );

    extensionMotorConfig.encoder.positionConversionFactor(MotorConstants.kClimberExtensionConversionFactor);
    climberMotorConfig.encoder.positionConversionFactor(MotorConstants.kClimberConversionFactor);

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  public Command setClimberPositionCommand(double climberExtensionMillimeters) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the climber's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedClimberExtension = MathUtil.clamp(
      climberExtensionMillimeters, 
      0, 
      MotorConstants.kClimberMaxExtensionMM
    ); // Adjust me!

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command climberPositionCommand = runOnce(() -> {
      // Pass in our clamped value as the arguement.
      climberCLC.setSetpoint(clampedClimberExtension, ControlType.kPosition);
    });

    // Return the ClimberPositionCommand object
    return climberPositionCommand;
  }


  // Direct and command methods to set the lock state
  private void setLock(boolean locked) { lockSolenoid.set(locked ? false : true); }
  public Command setLockCommand(boolean locked) { return runOnce(() -> setLock(locked)); }

  // Command to toggle the lock
  public Command toggleLockCommand() { return runOnce(() -> lockSolenoid.toggle()); }

  // Direct and command methods to actuate (extend and retract) the climber
  private void actuate(boolean extend) { extensionSolenoid.set(extend ? true : false); }
  public Command actuateCommand(boolean extend) { return runOnce(() -> actuate(extend)); }

  // Command to toggle the climber actuation (extend and retract)
  public Command toggleExtensionCommand() { return runOnce(() -> extensionSolenoid.toggle()); }

  // Primary command for the endgame climb
  public Command executeClimbSequenceCommand() {

    // Reusable sequence to climb a full rung
    Command climbFullRungCommand = new SequentialCommandGroup(
      setClimberPositionCommand(latchPosition),
      new WaitUntilCommand(() -> climberMotorPosition.get() + climberPositionBias >= latchPosition),
      setClimberPositionCommand(homePosition),
      new WaitUntilCommand(() -> climberMotorPosition.get() - climberPositionBias <= homePosition)
    );

    Command commandSequence = new SequentialCommandGroup(

      setLockCommand(false), // Set the lock state to be unlocked.
      climbFullRungCommand, // First rung stage (Starting from the ground).
      climbFullRungCommand, // Second rung stage (at this point, the ladder hooks are on the first rung).

      // Third run stage (at this point, the ladder hooks are on the second rung).
      // This stage will not engage the ladder hooks, it will go about half way before
      // engaging the lock.
      setClimberPositionCommand(lockPosition),
      new WaitUntilCommand(() -> climberMotorPosition.get() + climberPositionBias >= lockPosition),

      setLockCommand(true) // Engage the pneumatic lock
    );

    return commandSequence;
  }

  // Command for partial auto climb
  public Command executeAutoClimbSequenceCommand() {

    // Sequence to climb and then lock
    Command climbPartialRungCommand = new SequentialCommandGroup(
      setClimberPositionCommand(lockPosition),
      new WaitUntilCommand(() -> climberMotorPosition.get() + climberPositionBias >= lockPosition),
      setLockCommand(true)
    );

    return climbPartialRungCommand;
  }

  // Command to disengage the climb in teleop (only for the auto climb)
  public Command disengageAutoClimb() {

    Command disengageCommand = new SequentialCommandGroup(
      setLockCommand(false),
      setClimberPositionCommand(homePosition),
      new WaitUntilCommand(() -> climberMotorPosition.get() - climberPositionBias <= homePosition),
      setLockCommand(true),
      actuateCommand(false)
    ).onlyIf( // Only run this command if the robot looks like it climbed during auto
      () -> climberMotorPosition.get() >= homePosition + climberPositionBias);

    return disengageCommand;
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
