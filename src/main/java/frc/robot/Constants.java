// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    // Whole class is static, meaning it won't be used for creating objects.
    // Whole class is final, meaning values cannot be changed.
    public class ActuatorConstants {

        // Shared pneumatic hub CAN
        public static final int kPneumaticHubCANID = 5;

        // Use these to adjust global current limits across most motors
        public static final int kCommonNeoCurrentLimit = 40;
        public static final int kCommonNeo550CurrentLimit = 30;
        public static final int kvortexCurrentLimit = 40;  //change this


        // Shooter Constants
        public static final int kFlywheelCANID = 6;
        public static final int kBonnetCANID = 7;
        public static final int leftVortexCANID = 15; //change this
        public static final int rightVortexCANID = 14; //change this

        public static final boolean kFlywheelInverted = false;
        public static final boolean kBonnetInverted = false;

        public static final double kBonnetConversionFactor = 0.193911189;
        public static final double kFlywheelConversionFactor = 1.0;

        public static final double kFlywheelMaxRPM = 5000;
        public static final double kBonnetMaxExtensionDegrees = 15;

        public static final double[] kFlywheelPID = {0.0006, 0.000001, 0.06, 0.001};
        public static final double[] kBonnetPID = {0.1, 0, 0};

        public static final double kFlywheelSpinUpTime = 1.5;

        public static final double[][] shooterPresets  = {
            // [min,max,angle,speed]
            {1.72, 2.308, 0, 1000}, 
            {2.309, 2.6, 5, 1000}, 
            {2.61, 2.83, 4, 1250}, 
            {2.84, /*3.4*/ 20, 5, 1400}
            
        };

        

        

        // Hopper Constants
        public static final int kBeltCANID = 8;
        public static final int kCorralCANID = 9;
        public static final int kKickerCANID = 10;

        public static final boolean kBeltInverted = false;
        public static final boolean kCorralInverted = false;
        public static final boolean kKickerInverted = true;

        public static final double kKickerConversionFactor = 1.0; // May need changed

        public static final int kKickerMotorMaxRPM = 500; // Change Me!
        public static final double[] kKickerPID = {0.005, 0, 0}; // Tune me!


        // Intake Constants
        public static final int kintakeExtensionChannel = 14;
        public static final int kintakeRetractionChannel = 15;
        public static final int kRollerCANID = 11;

        public static final int kIntakeMotorCANID = 0;  //change this
        public static final int kExtendingMotorCANID = 0;  //change this

        public static final double[] kIntakeMotorPID = {0, 0, 0};  //change this
        public static final double[] kExtendingMotorPID = {0, 0, 0};  //change this

        public static final double RetractedPosition = 0; //change this
        public static final double ExtendedPosition = 0; //change this

        public static final double intakingSpeed = 0;  //put value 0-1; change this

        public static final boolean kRollerInverted = false; // May need changed!


        // Climber Constants
        public static final int kClimberCANID = 12; // Change me!
        public static final int kClimberExtensionChannel = 1; // Change me!
        public static final int kLockSolenoidChannel = 2; // Change me!

        public static final boolean kClimberInverted = false; // May need changed!

        public static final double kClimberConversionFactor = 1.0; // Change me!

        public static final int kClimberMaxExtensionMM = 50;

        public static final double[] kClimberPID = {0.005, 0, 0,}; // Tune me!
    }


    public class VisionConstants {

        // Pose estimator arguments
        public static final String kCamera1Name = "stellarvision";
        public static final String kCamera2Name = "stellarvision2";
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static final Transform3d kRobotToCam1 = new Transform3d(
            new Translation3d(Units.inchesToMeters(11.75-0.75), Units.inchesToMeters(12-0.68), Units.inchesToMeters(9.5)), 
            new Rotation3d(0, Units.degreesToRadians(360-34), Units.degreesToRadians(360 - 20))
        );
        public static final Transform3d kRobotToCam2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(11.75), Units.inchesToMeters(-12), Units.inchesToMeters(9.562)), 
            new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(360-90))
        );

        // Standard Deviation Constraints
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }


    public class MiscConstants {

        // Driver deadband
        public static final double kDriverDeadband = 0.1;

        // Whether pathplanner should be used
        public static final boolean kUsePathplanner = true;

        // Extend intake on enable
        public static final boolean kTeleopExtendIntake = true;

        // Controller Ports
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;

        // Hub positions
        public static final Pose2d kRedHubPosition = new Pose2d(
            11.925, 
            4.04, 
            new Rotation2d()
        );
        public static final Pose2d kBlueHubPosition = new Pose2d(
            4.625, 
            4.04, 
            new Rotation2d()
        );

        public static final double shootingHeight = 6;   
        //controls target max height. this is not to scale so you should use the desmos. it is the "B" slider

        public static final double hubHeight = 6;
        //this is the hub height in feet

        public static final double speedMultiplier = 60;
        //this is the multiplier that the speed is calculated with
    }
}
