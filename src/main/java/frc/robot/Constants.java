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
        public static final int kPneumaticHubCANID = 5; // Change Me!

        // Use these to adjust global current limits across most motors
        public static final int kCommonNeoCurrentLimit = 40;
        public static final int kCommonNeo550CurrentLimit = 30;


        // Shooter Constants
        public static final int kFlywheelCANID = 6; // Change me!
        public static final int kBonnetCANID = 7; // Change me!

        public static final boolean kFlywheelInverted = false; // May need changed!
        public static final boolean kBonnetInverted = false; // May need changed!

        public static final double kBonnetConversionFactor = 1.0; // Change me!
        public static final double kFlywheelConversionFactor = 1.0; // Change me!

        public static final double kFlywheelMaxRPM = 5000;
        public static final double kBonnetMaxExtensionMM = 0.3; // Change me!

        public static final double[] kFlywheelPID = {0.005, 0, 0.001};
        public static final double[] kBonnetPID = {0.03, 0, 0}; // Tune me!


        // Hopper Constants
        public static final int kBeltCANID = 8; // Change me!
        public static final int kCorralCANID = 9; // Change me!
        public static final int kKickerCANID = 10; // Change me!

        public static final boolean kBeltInverted = false; // May need changed!
        public static final boolean kCorralInverted = false; // May need changed!
        public static final boolean kKickerInverted = false; // May need changed!

        public static final double kKickerConversionFactor = 1.0; // Change me!

        public static final int kKickerMotorMaxRPM = 500; // Change Me!
        public static final double[] kKickerPID = {0.005, 0, 0}; // Tune me!


        // Intake Constants
        public static final int kintakeExtensionChannel = 0; // Change me!
        public static final int kRollerCANID = 11; // Change me!

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
