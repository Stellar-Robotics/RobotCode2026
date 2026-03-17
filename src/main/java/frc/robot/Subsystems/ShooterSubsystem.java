// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MiscUtils;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Subsystems.swerve.SwerveSubsystem;

public class ShooterSubsystem extends SubsystemBase {

  // Create motor (motor controller) objects.
  SparkMax flywheelMotor = new SparkMax(ActuatorConstants.kFlywheelCANID, MotorType.kBrushless);
  SparkMax bonnetMotor = new SparkMax(ActuatorConstants.kBonnetCANID, MotorType.kBrushless);

  // Store refrences to the motors' closed loop controllers.
  SparkClosedLoopController flywheelCLC = flywheelMotor.getClosedLoopController();
  SparkClosedLoopController bonnetCLC = bonnetMotor.getClosedLoopController();

  SwerveSubsystem swerveSubsystem;
  /** Creates a new Shooter. */
  public ShooterSubsystem(SwerveSubsystem swerveObject) {

    swerveSubsystem = swerveObject;

    // Create configuration objects for the motor controllers.
    SparkMaxConfig flywheelMotorConfig = new SparkMaxConfig();
    SparkMaxConfig bonnetMotorConfig = new SparkMaxConfig();

    // Set configuration options by calling methods on the configuration objects.
    flywheelMotorConfig
        .inverted(ActuatorConstants.kFlywheelInverted)
        .smartCurrentLimit(ActuatorConstants.kCommonNeoCurrentLimit).closedLoop.pid(ActuatorConstants.kFlywheelPID[0],
            ActuatorConstants.kFlywheelPID[1], ActuatorConstants.kFlywheelPID[2]).feedForward
        .kV(ActuatorConstants.kFlywheelPID[3]);
    bonnetMotorConfig
        .inverted(ActuatorConstants.kBonnetInverted)
        .smartCurrentLimit(ActuatorConstants.kCommonNeo550CurrentLimit).closedLoop
        .pid(ActuatorConstants.kBonnetPID[0], ActuatorConstants.kBonnetPID[1], ActuatorConstants.kBonnetPID[2]);

    // (NOTE: Methods Below Require These To Be Set Correctly)
    // Set conversion factors (adjust so it corresponds with millimeters).
    bonnetMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kBonnetConversionFactor);
    // Set so we get accurate conversion of RPMs at the flywheel.
    flywheelMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kFlywheelConversionFactor);

    // Call the configure method on the motor objects in order to apply the config
    // objects.
    flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bonnetMotor.configure(bonnetMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Method that stops the flywheel motor
  public void stopShooter() {
    flywheelMotor.stopMotor();
  }

  public void setFlywheelSpeed(double flywheelSpeedRPMs) {

    // Clamp our specified speed to a safe range
    double clampedRPM = MathUtil.clamp(flywheelSpeedRPMs, -ActuatorConstants.kFlywheelMaxRPM,
        ActuatorConstants.kFlywheelMaxRPM);

    // sets the closed loop controller to the velocity (RPMs) specified in the parameter.
      flywheelCLC.setSetpoint(clampedRPM, ControlType.kVelocity);
  }

  // Method that returns a command to set the target velocity of the flywheel
  // shaft.
  public Command setFlywheelSpeedCommand(double flywheelSpeedRPMs) {

    // Packages the setFlywheelSpeed method into a command
    Command flywheelCommand = runOnce(() -> setFlywheelSpeed(flywheelSpeedRPMs));

    // Set the command name and return the flywheelCommand object.
    flywheelCommand.setName("SetFlywheelTo" + MathUtil.clamp(flywheelSpeedRPMs, -ActuatorConstants.kFlywheelMaxRPM,
      ActuatorConstants.kFlywheelMaxRPM) + "RPM");
    return flywheelCommand;
  }

  public void setBonnetPosition(double bonnetExtensionDegrees) {

    // Run our parameter through a clamp algorithm to make sure
    // we can't accidentally extend past the bonnet's mechanical limits.
    // We'll then store it in a new variable called clampedPositionRotations.
    double clampedBonnetExtension = MathUtil.clamp(bonnetExtensionDegrees, 0,
      ActuatorConstants.kBonnetMaxExtensionDegrees);

    // Pass in our clamped value as the arguement.
    bonnetCLC.setSetpoint(clampedBonnetExtension, ControlType.kPosition);
  }

  // Method that returns a command to set the target extension of the bonnet.
  public Command setBonnetPositionCommand(double bonnetExtensionDegrees) {

    // Create a command with an anonymous method that sets the target
    // position using the closed loop controller.
    Command bonnetPositionCommand = runOnce(() -> setBonnetPosition(bonnetExtensionDegrees));

    // Set the command name and return the bonnetPositionCommand object
    bonnetPositionCommand.setName("SetBonnetTo" + MathUtil.clamp(bonnetExtensionDegrees, 0,
      ActuatorConstants.kBonnetMaxExtensionDegrees) + "Deg");
    return bonnetPositionCommand;
  }

  public Command aimByDistance(boolean redHub) {

    Translation2d targetHub;
    
    if (redHub) {
      targetHub = new Translation2d( // this makes a point of the red hub
          ActuatorConstants.redHubPosition[0], ActuatorConstants.redHubPosition[1]);
    } else {
      targetHub = new Translation2d( // this makes a point of the blue hub
        ActuatorConstants.blueHubPosition[0], ActuatorConstants.blueHubPosition[1]);
    }

    Supplier<Translation2d> currentRobotPoint = () -> new Translation2d( // this gets the robot position and makes a point of it
      swerveSubsystem.getOdometryEstimate().getX(), swerveSubsystem.getOdometryEstimate().getY());

    Supplier<Double> distance = () -> targetHub.getDistance(currentRobotPoint.get()); // finds the distance of the between the robot and hub

    return runEnd(
      () -> setBonnetPosition(90 -Units.radiansToDegrees(Math.atan(Units.inchesToMeters(132.36) / distance.get()))),
      () -> setBonnetPosition(0)
    );
  }


  public void setShooterProfile(double speedRPM, double bonnetDegrees) {
    setFlywheelSpeed(speedRPM);
    setBonnetPosition(bonnetDegrees);
  }

  public Command setShooterProfileCommand(double speedRPM, double bonnetDegrees) {
    return runOnce(() -> setShooterProfile(speedRPM, bonnetDegrees));
  }


  public Supplier<Double> distance() {
    boolean redHub = MiscUtils.isRedAlliance().getAsBoolean();
    Translation2d targetHub;
      
    if (redHub) {
      targetHub = new Translation2d( // this makes a point of the red hub
          ActuatorConstants.redHubPosition[0], ActuatorConstants.redHubPosition[1]);
    } else {
      targetHub = new Translation2d( // this makes a point of the blue hub
        ActuatorConstants.blueHubPosition[0], ActuatorConstants.blueHubPosition[1]);
    }

    Supplier<Translation2d> currentRobotPoint = () -> new Translation2d( // this gets the robot position and makes a point of it
      swerveSubsystem.getOdometryEstimate().getX(), swerveSubsystem.getOdometryEstimate().getY());

    Supplier<Double> distance = () -> targetHub.getDistance(currentRobotPoint.get()); // finds the distance of the between the robot and hub

    return distance;
  }

  public void autoAim() {

    Supplier<Double> distance = distance(); // finds the distance of the between the robot and hub

    for (double[] preset : ActuatorConstants.shooterPresets) {
      if (distance().get() >= preset[0] && distance.get() <= preset[1]) {
        setShooterProfile(preset[3], preset[2]);
        break;
      } else {
        setShooterProfile(0, 0);
        break;
      }
    }
  }

  public Command autoAimInstantCommand() { return runOnce(() -> autoAim()); }
  public Command autoAimRunCommand() { return runEnd(() -> autoAim(), () -> setShooterProfile(0, 0)); }


  

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DistanceFromHub", distance().get());
  }
}
