// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MiscUtils;
import frc.robot.Constants.MiscConstants;
import frc.robot.StellarHID.StellarHID;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {

  // Enable/Disable vision/odometry updates
  boolean visionUpdates = true;
  boolean usePathplanner = false;

  // Class accessable objects
  private SwerveDrive swerveDrive;
  private Vision vision;

  private PIDController absoluteAnglePID = new PIDController(0.015, 0, 0);


  public SwerveSubsystem(String configDirectory) {

    // The limiting speed of the drive train
    double maxSpeed = Units.feetToMeters(25);

    absoluteAnglePID.enableContinuousInput(-180, 180);
    SmartDashboard.putNumber("TranslationSpeed", 4.8);
    SmartDashboard.putNumber("RotationSpeed", 4.0);

    // Set desired level of debugging verbosity for the swerve system
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // Obtain a directory refrence to the swerve config files and create a YAGSL SwerveDrive object with it.
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), configDirectory);
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maxSpeed);
      System.out.println("INFO: Successfully created swerve object from JSON!");
    } catch (Exception e) {
      System.out.println("FATAL ERROR: Failed to create swerve object from JSON!");
      e.printStackTrace();
      throw new RuntimeException();
    }

    // Smart dashboard params
    SmartDashboard.putNumber("SetPoint", 0);

    // Set some conditions for the swerve systems
    swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.

    // Initialize vision system
    if (visionUpdates) {
      vision = new Vision(swerveDrive::addVisionMeasurement);
      swerveDrive.stopOdometryThread(); // We'll manually update along with vision for better syncronization.
    }

    // Initialize PathPlanner AutoBuilder
    if (usePathplanner) {
      initPathPlanner();
    }
  }


  // Initialize PathPlanner
  public void initPathPlanner() {

    RobotConfig config;

    try {

      config = RobotConfig.fromGUISettings(); // Attempt to load from the GUI prefrences
      final boolean enableFeedforward = true;

      AutoBuilder.configure(

        swerveDrive::getPose,
        swerveDrive::resetOdometry,
        swerveDrive::getRobotVelocity,

        (speedsBotRel, feedForwards) -> {
          if (enableFeedforward) {
            swerveDrive.drive(
              speedsBotRel,
              swerveDrive.kinematics.toSwerveModuleStates(speedsBotRel),
              feedForwards.linearForces()
            );
          } else {
            swerveDrive.setChassisSpeeds(speedsBotRel);
          }
        },

        new PPHolonomicDriveController (
            new PIDConstants(10, 0.0, 0.0),
            new PIDConstants(5, 0.0, 0.0)
        ),

        config,
        MiscUtils.isRedAlliance(),
        this

      );

    } catch (Exception e) {
      System.out.println("ERROR: Failed to obtain Path Planner config from GUI!");
      e.printStackTrace();
    }

    // Avoid delays by preloading the pathfinding functionality
    // NOTE: Custom path following commands should be put before this.
    PathfindingCommand.warmupCommand().schedule();
  }


  // Get the YAGSL swerveDrive object
  public SwerveDrive getSwerveDrive() { return swerveDrive; }

  // Get the robot's current position esimate
  public Pose2d getOdometryEstimate() { return swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition(); }


  // Drive relative to the robots frame of refrence
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    
    swerveDrive.setChassisSpeeds(robotRelativeSpeeds);
  }


  // Cross all modules to keep the chassis from moving
  public void lock() {

    swerveDrive.lockPose();
  }


  // Zero the gyro to the red aliance station
  public void zeroGyro() {

    swerveDrive.zeroGyro();
  }


  // Drive relative to the coordinates on the field
  public void driveFieldOriented(ChassisSpeeds fieldRelativeSpeeds) {

    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, swerveDrive.getYaw());
    swerveDrive.setChassisSpeeds(robotRelativeSpeeds);
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    if (visionUpdates) {

      swerveDrive.updateOdometry();
      vision.periodic();
    }
  }

  /* -------------------------
  * Command methods
  * ------------------------- */

  // Command to drive relative to the coordinates on the field
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {

    // Create command
    Command driveCommand = run(
      () -> {
        swerveDrive.driveFieldOriented(velocity.get());
      }
    );
    
    // Name and return
    driveCommand.setName("DriveFieldOriented");
    return driveCommand;
  }


  // A method for using the custom Stellar Controller with YAGSL
  public Command stellarCTRLDriveCommand(StellarHID controller) {

    return run(() -> {

      // Controller raw input
      double[] inputsWithDB = MiscUtils.circularDeadband(-controller.getLeftX(), controller.getLeftY(), 0.2);
      double transX = inputsWithDB[0];
      double transY =  inputsWithDB[1];
      Rotation2d rightRotaryAngle = controller.getRightRotary();

      // Current and desired robot angles (range of 0:360)
      double currentAngle = MathUtil.inputModulus(swerveDrive.getYaw().getDegrees(), 0, 360);
      double desiredAngle = rightRotaryAngle.getDegrees();

      // rotary PID Calculations
      double diffRot = controller.calcRotaryPID(currentAngle, desiredAngle, 0);

      // Obtain the base speed multipliers
      double dashTranslationSpeed = SmartDashboard.getNumber("TranslationSpeed", 4.8);
      double dashAngularSpeed =  SmartDashboard.getNumber("RotationSpeed", 4.0);

      // This can be manipulated in logic to facilitate speed mode control
      double xSpeedDelivered = transX * dashTranslationSpeed;
      double ySpeedDelivered = transY * dashTranslationSpeed;
      double rotDelivered = diffRot * dashAngularSpeed;

      // Create chassis speed object and issue it to the subsystem
      ChassisSpeeds positionCommanded = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
      driveFieldOriented(positionCommanded);

      // Report some telemetry to the dashboard
      SmartDashboard.putNumber("RightRotaryRawValue", controller.getRawRightEncoderValue());
      SmartDashboard.putNumber("RightRotaryAngleValue", controller.getRightRotary().getDegrees());

      SmartDashboard.putNumber("RotaryDesired", desiredAngle);
      SmartDashboard.putNumber("RotaryPosition", currentAngle);

      SmartDashboard.putNumber("SwerveYaw", swerveDrive.getYaw().getDegrees());
    });
  }


  /**
   * An all in one method to control the swerve via translational velocities and an absolute angle
   * 
   * @param radiusX radial/translational velocity on the X axis (range from -1 to 1)
   * @param radiusY radial/translational velocity on the Y axis (range from -1 to 1)
   * @param yaw absolute desired yaw angle (range from -180 to 180)
   * @param deadband cutoff point where the value is no longer registered (set 0 for none)
   * 
   * @return command object to drive the robot chassis
   */
  public Command driveFieldOrientedWithAbsoluteYaw(Supplier<Double> radiusX, Supplier<Double> radiusY, Supplier<Rotation2d> yaw, double deadband) {

    // Create command object to contain our drive code
    Command driveCommand = run(() -> {

      // Apply a deadband to translational inputs
      double[] filteredTranslation = MiscUtils.circularDeadband(radiusX.get(), radiusY.get(), deadband);
      double velocityX = filteredTranslation[0];
      double velocityY = filteredTranslation[1];

      // Obtain current and desired angles (Map continuous robot angle to a wrapping standard range)
      double robotAngle = MathUtil.inputModulus(swerveDrive.getYaw().getDegrees(), -180, 180);
      double desiredAngle = yaw.get().getDegrees();

      // With a PID controller, calculate the angular velocity to align the robot with the controller angle
      double angularVelocity = absoluteAnglePID.calculate(robotAngle, desiredAngle);

      // Obtain the base speed multipliers
      double dashTranslationSpeed = SmartDashboard.getNumber("TranslationSpeed", 4.8);
      double dashAngularSpeed =  SmartDashboard.getNumber("RotationSpeed", 4.0);

      // Create a chassis speeds object from the information above, and then
      // convert it so that it's field oriented.
      ChassisSpeeds targetSpeeds = new ChassisSpeeds(velocityX * dashTranslationSpeed, velocityY * dashTranslationSpeed, angularVelocity * dashAngularSpeed);
      driveFieldOriented(targetSpeeds);

      // Report some telemetry to the dashboard

      SmartDashboard.putNumber("RotaryDesired", desiredAngle);
      SmartDashboard.putNumber("RobotPosition", robotAngle);

      SmartDashboard.putNumber("SwerveYaw", swerveDrive.getYaw().getDegrees());

      swerveDrive.field.getObject("HubTarget").setPose(MiscConstants.kRedHubPosition);
    });

    // return the command
    return driveCommand;
  }


  /**
   * An all in one method to control the swerve via translational velocities and an absolute angle
   * 
   * @param radiusX radial/translational velocity on the X axis (range from -1 to 1)
   * @param radiusY radial/translational velocity on the Y axis (range from -1 to 1)
   * @param yaw absolute desired yaw angle (range from -180 to 180)
   * @param deadband cutoff point where the value is no longer registered (set 0 for none)
   * 
   * @return command object to drive the robot chassis
   */
  public Command driveFieldOrientedWithOrbit(Supplier<Double> radiusX, Supplier<Double> radiusY, Supplier<Rotation2d> yaw, double deadband) {

    // Create command object to contain our drive code
    Command driveCommand = run(() -> {

      // Apply a deadband to translational inputs
      double[] filteredTranslation = MiscUtils.circularDeadband(radiusX.get(), radiusY.get(), deadband);
      double velocityX = filteredTranslation[0];
      double velocityY = filteredTranslation[1];

      // Obtain current and desired angles (Map continuous robot angle to a wrapping standard range)
      double robotAngle = MathUtil.inputModulus(swerveDrive.getOdometryHeading().getDegrees(), -180, 180);
      double desiredAngle = yaw.get().getDegrees();

      // With a PID controller, calculate the angular velocity to align the robot with the controller angle
      double angularVelocity = absoluteAnglePID.calculate(robotAngle, robotAngle + desiredAngle);

      // Obtain the base speed multipliers
      double dashTranslationSpeed = SmartDashboard.getNumber("TranslationSpeed", 4.8);
      double dashAngularSpeed =  SmartDashboard.getNumber("RotationSpeed", 4.0);

      // Create a chassis speeds object from the information above, and then
      // convert it so that it's field oriented.
      ChassisSpeeds targetSpeeds = new ChassisSpeeds(velocityX * dashTranslationSpeed, velocityY * dashTranslationSpeed, angularVelocity * dashAngularSpeed);
      driveFieldOriented(targetSpeeds);

      // Report some telemetry to the dashboard

      SmartDashboard.putNumber("RotaryDesired", desiredAngle);
      SmartDashboard.putNumber("RobotPosition", robotAngle);

      SmartDashboard.putNumber("SwerveYaw", swerveDrive.getYaw().getDegrees());

      swerveDrive.field.getObject("HubTarget").setPose(MiscConstants.kRedHubPosition);
    });

    // return the command
    return driveCommand;
  }


  // Use PathPlanner to path find to a position
  public Command driveToPose(Pose2d pose) {

    // Path finding constraints
    PathConstraints constraints = new PathConstraints(
      swerveDrive.getMaximumChassisVelocity(), 
      4.0,
      swerveDrive.getMaximumChassisAngularVelocity(), 
      Units.degreesToRadians(720)
    );

    return AutoBuilder.pathfindToPose(
      pose,
      constraints,
      edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }
}