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


        // Shooter Constants
        public static final int kFlywheelCANID = 6;
        public static final int kBonnetCANID = 7;

        public static final boolean kFlywheelInverted = false;
        public static final boolean kBonnetInverted = false;

        public static final double kBonnetConversionFactor = 0.193911189;
        public static final double kFlywheelConversionFactor = 1.0;

        public static final double kFlywheelMaxRPM = 5000;
        public static final double kBonnetMaxExtensionDegrees = 21;

        public static final double[] kFlywheelPID = {0.0005, 0, 0.04, 0.015};
        public static final double[] kBonnetPID = {0.1, 0, 0};

        public static final double[] redHubPosition = {Units.inchesToMeters(464.5), Units.inchesToMeters(161.7)};
        public static final double[] blueHubPosition = {Units.inchesToMeters(186.5), Units.inchesToMeters(161.7)};


        // Hopper Constants
        public static final int kBeltCANID = 8;
        public static final int kCorralCANID = 9;
        public static final int kKickerCANID = 10;

        public static final boolean kBeltInverted = false;
        public static final boolean kCorralInverted = true;
        public static final boolean kKickerInverted = true;

        public static final double kKickerConversionFactor = 1.0; // May need changed

        public static final int kKickerMotorMaxRPM = 500; // Change Me!
        public static final double[] kKickerPID = {0.005, 0, 0}; // Tune me!


        // Intake Constants
        public static final int kintakeExtensionChannel = 14;
        public static final int kintakeRetractionChannel = 15;
        public static final int kRollerCANID = 11;

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
        public static final String kCameraName = "stellarvision";
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(-7), Units.inchesToMeters(11.3), 0), 
            new Rotation3d(0, Units.degreesToRadians(292), 0)
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

        // Hub positions
        public static final Pose2d kRedHubPosition = new Pose2d(
            Units.inchesToMeters(464.5), 
            Units.inchesToMeters(161.7), 
            new Rotation2d()
        );
        public static final Pose2d kBlueHubPosition = new Pose2d(
            Units.inchesToMeters(186.5), 
            Units.inchesToMeters(161.7), 
            new Rotation2d()
        );
    }
}
