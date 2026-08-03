// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.MiscUtils;
import frc.robot.Constants.MiscConstants;

import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase {

  // Enable/Disable vision/odometry updates
  boolean visionUpdates = true;

  // Class accessable objects
  private SwerveDrive swerveDrive;
  private Vision vision;

  private PIDController absoluteAnglePID = new PIDController(0.015, 0, 0);
  private double lastVelocity;


  // SwerveSubsystem constructor
  public SwerveSubsystem(String configDirectory) {

    // The limiting speed of the drive train
    double maxSpeed = Units.feetToMeters(25);

    absoluteAnglePID.enableContinuousInput(-180, 180);
    SmartDashboard.putNumber("TranslationSpeed", 1);
    SmartDashboard.putNumber("RotationSpeed", 4);

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
    swerveDrive.setCosineCompensator(!RobotBase.isSimulation()); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true, false, 0.1); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.resetOdometry(new Pose2d(
      0.3, 
      0.3, 
      Rotation2d.fromDegrees(0))
    );
    // swerveDrive.resetOdometry(new Pose2d(
    //   3.574, 
    //   4.032, 
    //   Rotation2d.fromDegrees(0))
    // );

    // Initialize vision system
    if (visionUpdates) {
      vision = new Vision(swerveDrive::addVisionMeasurement);
      swerveDrive.stopOdometryThread(); // We'll manually update along with vision for better syncronization.
    }

    // Initialize PathPlanner AutoBuilder
    // if (MiscConstants.kUsePathplanner) {
    //   initPathPlanner();
    // }

    SmartDashboard.putNumber("robotTravel", getOdometryEstimate().getX());
  }


  /* -------
  * Methods
  --------- */

  // Get the YAGSL swerveDrive object
  public SwerveDrive getSwerveDrive() { return swerveDrive; }


  // Get the robot's current position esimate
  public Pose2d getOdometryEstimate() { return swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition(); }


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
            new PIDConstants(12, 1, 0),
            new PIDConstants(5, 0, 0)
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


  // Cross all modules to keep the chassis from moving
  public void lock() {

    swerveDrive.lockPose();
  }


  // Zero the gyro to the red aliance station
  public void zeroGyro() {

    swerveDrive.zeroGyro();
  }


  // Drive relative to the robots frame of refrence
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    
    swerveDrive.setChassisSpeeds(robotRelativeSpeeds);
  }


  // Drive relative to the coordinates on the field
  public void driveFieldOriented(ChassisSpeeds fieldRelativeSpeeds) {

    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, swerveDrive.getYaw());
    swerveDrive.setChassisSpeeds(robotRelativeSpeeds);
  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorAngularCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
        new Config(),
        this, 
        swerveDrive, 
        12, 
        true
      ),
      3.0,
      5.0, 
      3.0
    );
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorLinearCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
        new Config(),
        this, 
        swerveDrive, 
        12, 
        false
      ),
      3.0,
      5.0, 
      3.0
    );
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new Config(),
        this, 
        swerveDrive
      ),
      3.0,
      5.0, 
      3.0
    );
  }


  // Configures a trigger to stop if it exceeds a set boundry
  public void setLogicalBarrier() {

    // double[] boxDims = { // Obtain box dimensions from Dashboard
    //   SmartDashboard.getNumber("LengthRestrictionMeters", 2.5), // X - Downfield
    //   SmartDashboard.getNumber("WidthRestrictionMeters", 2.5) // Y - Left of downfield
    // };

    // Pose2d boxCenter = new Pose2d(
    //   (boxDims[0] / 2), 
    //   (boxDims[1] / 2), 
    //   Rotation2d.fromDegrees(0)
    // );

    // PathConstraints pathConsts = new PathConstraints( // About half of the speed in auto
    //   swerveDrive.getMaximumChassisVelocity(),
    //   2.0,
    //   swerveDrive.getMaximumChassisAngularVelocity(),
    //   Units.degreesToRadians(270));

    // Create dashboard params
    SmartDashboard.putBoolean("EnableBarrier", true);
    SmartDashboard.putNumber("BarrierLengthFeet", 8);
    SmartDashboard.putNumber("BarrierWidthFeet", 8);

    BooleanSupplier violConditions = () -> { // Returns true if in violation
      double odomEstX = getOdometryEstimate().getX();
      double odomEstY = getOdometryEstimate().getY();

      if (
        odomEstX <= Units.feetToMeters(SmartDashboard.getNumber("BarrierLengthFeet", 8)) &&
        odomEstY <= Units.feetToMeters(SmartDashboard.getNumber("BarrierWidthFeet", 8)) &&
        odomEstX >= 0 &&
        odomEstY >= 0
      ) {
        return false;
      } else {
        return true;
      }
    };

    // Inturrupt drive command if barrier is crossed.
    // Will command will only execute if barrier is enabled
    new Trigger(violConditions)
      .onTrue(
        run(() -> {})
          .onlyIf(() -> SmartDashboard.getBoolean("EnableBarrier", true))
          .withName("RobotStopOverride")
      );
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    if (visionUpdates) {

      swerveDrive.updateOdometry();
      vision.periodic();
      SmartDashboard.putBoolean("isRedAlliance", MiscUtils.isRedAlliance().getAsBoolean());
    }
    SmartDashboard.putNumber("robotTravel", getOdometryEstimate().getX());

    if (swerveDrive.getRobotVelocity().vxMetersPerSecond >= lastVelocity) {
      SmartDashboard.putNumber("MaxVelocity", swerveDrive.getRobotVelocity().vxMetersPerSecond);
      lastVelocity = swerveDrive.getRobotVelocity().vxMetersPerSecond;
    }
  }

  /* -------------------------
  * Command methods
  * ------------------------- */

  // Command to drive relative to the coordinates on the field
  public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {

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
  public void driveFieldOrientedWithAbsoluteYaw(Supplier<Double> radiusX, Supplier<Double> radiusY, Supplier<Rotation2d> yaw, double deadband) {
    // Apply a deadband to translational inputs
    double[] filteredTranslation = MiscUtils.circularDeadband(radiusX.get(), radiusY.get(), deadband);
    double velocityX = Math.copySign(Math.pow(filteredTranslation[0], 2), filteredTranslation[0]);
    double velocityY = Math.copySign(Math.pow(filteredTranslation[1], 2), filteredTranslation[1]);

    // Obtain current and desired angles (Map continuous robot angle to a wrapping standard range)
    double robotAngle = MathUtil.inputModulus(swerveDrive.getYaw().getDegrees(), -180, 180);
    double desiredAngle = yaw.get().getDegrees();

    // With a PID controller, calculate the angular velocity to align the robot with the controller angle
    double angularVelocity = absoluteAnglePID.calculate(robotAngle, desiredAngle + 180);

    // Obtain the base speed multipliers
    double dashTranslationSpeed = SmartDashboard.getNumber("TranslationSpeed", 4.8);
    double dashAngularSpeed =  SmartDashboard.getNumber("RotationSpeed", 4.0);

    // Create a chassis speeds object from the information above, and then
    // convert it so that it's field oriented.
    ChassisSpeeds targetSpeeds = new ChassisSpeeds(velocityX * dashTranslationSpeed, velocityY * dashTranslationSpeed, angularVelocity * dashAngularSpeed);
    driveFieldOriented(targetSpeeds);

    // Report some telemetry to the dashboard
    SmartDashboard.putNumber("DesiredHeading", desiredAngle);
    SmartDashboard.putNumber("RobotHeading", robotAngle);
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
  public Command driveFieldOrientedWithAbsoluteYawCommand(Supplier<Double> radiusX, Supplier<Double> radiusY, Supplier<Rotation2d> yaw, double deadband) {

    // Create command object to contain our drive code
    Command driveCommand = run(() -> driveFieldOrientedWithAbsoluteYaw(radiusX, radiusY, yaw, deadband));

    // return the command
    return driveCommand;
  }


  /**
   * Controlls the swerve chassis via translational velocities and a desired setpoint difference
   * 
   * @param radiusX radial/translational velocity on the X axis (range from -1 to 1)
   * @param radiusY radial/translational velocity on the Y axis (range from -1 to 1)
   * @param yaw the difference of the desired yaw angle from the robot (range from -180 to 180)
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
      SmartDashboard.putNumber("DesiredHeading", desiredAngle);
      SmartDashboard.putNumber("RobotHeading", robotAngle);

      Pose2d orbitTarget = MiscUtils.isRedAlliance().getAsBoolean() ? MiscConstants.kRedHubPosition : MiscConstants.kBlueHubPosition;
      swerveDrive.field.getObject("OrbitTarget").setPose(orbitTarget);
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


  // Robot dog mode - your loyal mechanical companion!
  //   - or bodyguard, depending on how you view it.
  public Command followMe(double distanceMeters, int tagID) {

    // Setup PID Controllers and configure turning PID
    @SuppressWarnings("resource")
    PIDController xPID = new PIDController(0.001, 0, 0); // Tune me
    @SuppressWarnings("resource")
    PIDController rotPID = new PIDController(0.0001, 0, 0); // Tune me
    rotPID.enableContinuousInput(-Math.PI, Math.PI);

    Command followCMD = run(() -> {
      // Get tag data and null check it
      Optional<PhotonTrackedTarget> tagData;
      if ((tagData = vision.getTag(false, tagID)) == null) { return; };
      if (tagData.isEmpty()) { return; }

      // Get position information
      Transform3d camToTgt = tagData.get().getBestCameraToTarget();
      // Calculate PID for distance
      double xVel = xPID.calculate(camToTgt.getX(), distanceMeters /* Assuming meters, but could be different */);
      // Calculate PID for yaw
      double rotVel = rotPID.calculate(camToTgt.getY(), 0 /* Assuming 0 is the center */);

      // Create a chassis speeds object from PID calc
      ChassisSpeeds targetSpeeds = new ChassisSpeeds(xVel, 0, rotVel);

      // Apply chassis speeds to robot
      driveRobotRelative(targetSpeeds);
    });

    return followCMD;
  }
}