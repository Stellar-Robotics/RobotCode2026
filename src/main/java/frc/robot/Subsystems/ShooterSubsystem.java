// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MiscUtils;
import frc.robot.Constants.ActuatorConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Subsystems.swerve.SwerveSubsystem;

public class ShooterSubsystem extends SubsystemBase {

  // Create motor (motor controller) objects.
  //SparkMax flywheelMotor = new SparkMax(ActuatorConstants.kFlywheelCANID, MotorType.kBrushless);
  SparkMax bonnetMotor = new SparkMax(ActuatorConstants.kBonnetCANID, MotorType.kBrushless);
  SparkFlex leftVortexMotor = new SparkFlex(ActuatorConstants.leftVortexCANID, MotorType.kBrushless);
  SparkFlex rightVortexMotor = new SparkFlex(ActuatorConstants.rightVortexCANID, MotorType.kBrushless);



  // Store refrences to the motors' closed loop controllers.
  //SparkClosedLoopController flywheelCLC = flywheelMotor.getClosedLoopController();
  SparkClosedLoopController bonnetCLC = bonnetMotor.getClosedLoopController();
  SparkClosedLoopController leftVortexCLC = leftVortexMotor.getClosedLoopController();
  SparkClosedLoopController rightVortexCLC = rightVortexMotor.getClosedLoopController();

  SwerveSubsystem swerveSubsystem;
  /** Creates a new Shooter. */
  public ShooterSubsystem(SwerveSubsystem swerveObject) {
    swerveSubsystem = swerveObject;

    // Create configuration objects for the motor controllers.
    SparkMaxConfig flywheelMotorConfig = new SparkMaxConfig();
    SparkMaxConfig bonnetMotorConfig = new SparkMaxConfig();
    SparkFlexConfig leftVortexMotorConfig = new SparkFlexConfig();
    SparkFlexConfig rightVortexMotorConfig = new SparkFlexConfig();

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

    rightVortexMotorConfig
      .inverted(false)
      .smartCurrentLimit(ActuatorConstants.kvortexCurrentLimit)
      .closedLoop.pid(ActuatorConstants.kFlywheelPID[0],
            ActuatorConstants.kFlywheelPID[1], ActuatorConstants.kFlywheelPID[2]).feedForward
        .kV(ActuatorConstants.kFlywheelPID[3]);

    leftVortexMotorConfig
      .inverted(false)
      .follow(ActuatorConstants.rightVortexCANID, true)
      .smartCurrentLimit(ActuatorConstants.kvortexCurrentLimit);

    

    // (NOTE: Methods Below Require These To Be Set Correctly)
    // Set conversion factors (adjust so it corresponds with millimeters).
    bonnetMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kBonnetConversionFactor);
    // Set so we get accurate conversion of RPMs at the flywheel.
    flywheelMotorConfig.encoder.positionConversionFactor(ActuatorConstants.kFlywheelConversionFactor);

    // Call the configure method on the motor objects in order to apply the config
    // objects.
    //flywheelMotor.configure(flywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bonnetMotor.configure(bonnetMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftVortexMotor.configure(leftVortexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightVortexMotor.configure(rightVortexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public Command test() {
    return runEnd(() -> rightVortexCLC.setSetpoint(2000, ControlType.kVelocity), () -> rightVortexMotor.set(0));
  }


  public void setVortexMotorSpeed(double flywheelSpeedRPMs) { //left vortex follows right

    // Clamp our specified speed to a safe range

    // double clampedRPM = MathUtil.clamp(flywheelSpeedRPMs, -ActuatorConstants.kFlywheelMaxRPM,
    //   ActuatorConstants.kFlywheelMaxRPM);

    double vortexClampedRPM = MathUtil.clamp(flywheelSpeedRPMs, -ActuatorConstants.kFlywheelMaxRPM, ActuatorConstants.kFlywheelMaxRPM);


    // sets the closed loop controller to the velocity (RPMs) specified in the parameter.
    
    //flywheelCLC.setSetpoint(clampedRPM, ControlType.kVelocity);
    rightVortexCLC.setSetpoint(vortexClampedRPM, ControlType.kVelocity);
  }

  public Command setVortexMotorSpeedCommand(double flywheelSpeedRPMs) {

    return runOnce(() -> setVortexMotorSpeed(flywheelSpeedRPMs));
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

    return runOnce(() -> setBonnetPosition(bonnetExtensionDegrees));
  }


  public Command aimByDistanceRunCommand() { // I need a different equation!

    return runEnd(
      () -> setBonnetPosition(angleFinder().get()),
      () -> setBonnetPosition(0)
    );
  }


  public void setShooterProfile(double speedRPM, double bonnetDegrees) {
    setVortexMotorSpeed(speedRPM);
    if (speedRPM <= 20) {
      rightVortexMotor.stopMotor();
      setBonnetPosition(bonnetDegrees);
    } else {
      setBonnetPosition(bonnetDegrees);
    }
  }

  public Command setShooterProfileCommand(double speedRPM, double bonnetDegrees) {

    return runOnce(() -> setShooterProfile(speedRPM, bonnetDegrees));
  }


  public Supplier<Double> getDistanceToHub() {

    boolean redHub = MiscUtils.isRedAlliance().getAsBoolean();
    Translation2d targetHub;
      
    if (redHub) {
      targetHub = new Translation2d( // this makes a point of the red hub
        MiscConstants.kRedHubPosition.getX(), MiscConstants.kRedHubPosition.getY());
    } else {
      targetHub = new Translation2d( // this makes a point of the blue hub
        MiscConstants.kBlueHubPosition.getX(), MiscConstants.kBlueHubPosition.getY());
    }

    Supplier<Translation2d> currentRobotPoint = () -> new Translation2d( // this gets the robot position and makes a point of it
      swerveSubsystem.getOdometryEstimate().getX(), swerveSubsystem.getOdometryEstimate().getY());

    Supplier<Double> distance = () -> targetHub.getDistance(currentRobotPoint.get()); // finds the distance of the between the robot and hub

    return distance;
  }

  public Supplier<List<Double>> vertex() {
    Supplier<Double> distance = getDistanceToHub();

    Supplier<Double> shootingVertex = () -> 3 * MiscConstants.shootingHeight / distance.get();

    Supplier<Double> xAxisInterception = () -> shootingVertex.get() * distance.get() + MiscConstants.hubHeight / Math.pow(distance.get(), 2);

    Supplier<Double> additionalPointX = () -> distance.get() - 0.1;

    Supplier<Double> additionalPointY = () -> (
      -xAxisInterception.get() * Math.pow(additionalPointX.get(), 2) + 
      shootingVertex.get() * additionalPointX.get() + 
      MiscConstants.hubHeight);

    //Supplier<Double[]> vertex = () -> {additionalPointX.get(), additionalPointY.get()};

    Supplier<List<Double>> vertex = () -> Arrays.asList(additionalPointX.get(), additionalPointY.get());

    return vertex;
  }

  public Supplier<Double> angleFinder() {
    double[] vertex = {vertex().get().get(0), vertex().get().get(1)};

    Supplier<Double> distance = getDistanceToHub();

    Supplier<Double> slope = () -> vertex[1] - distance.get() / vertex[0];


    return () -> 90 - Math.atan(slope.get());

  }

  public double speedFinder() {
    double[] vertex = {vertex().get().get(0), vertex().get().get(1)};

    double velocity = Math.sqrt(Units.feetToMeters(vertex[1]) * 2 * 9.8);

    double speed  = velocity * MiscConstants.speedMultiplier;

    return speed;
  }


  public void autoAim() {

    //setShooterProfile(speedFinder(), angleFinder().get());

    for (double[] preset : ActuatorConstants.shooterPresets) {
      if (getDistanceToHub().get() >= preset[0] && getDistanceToHub().get() <= preset[1]) {
        setShooterProfile(preset[3], preset[2]);
        break;
      } else {
        setShooterProfile(0, 0);
        break;
      }
    }
  }


  


  public Command autoAimInstantCommand() { return runOnce(() -> autoAim()); }
  public Command autoAimRunCommand() { return runEnd(() -> autoAim(), () -> setShooterProfile(speedFinder(), angleFinder().get())); }


  @Override
  public void periodic() {

    SmartDashboard.putNumber("DistanceFromHub", getDistanceToHub().get());
    SmartDashboard.putNumber("bonnetAngle", angleFinder().get());
  }
}
