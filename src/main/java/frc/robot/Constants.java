// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    // Whole class is static, meaning it won't be used for creating objects.
    // Whole class is final, meaning values cannot be changed.
    public class MotorConstants {

        // Use these to adjust global current limits across most motors
        public static final int kCommonNeoCurrentLimit = 40;
        public static final int kCommonNeo550CurrentLimit = 30;


        // Shooter Constants
        public static final int kFlywheelCANID = 5; // Change me!
        public static final int kBonnetCANID = 6; // Change me!

        public static final boolean kFlywheelInverted = false; // May need changed!
        public static final boolean kBonnetInverted = false; // May need changed!

        public static final double kBonnetConversionFactor = 0; // Change me!
        public static final double kFlywheelConversionFactor = 0; // Change me!

        public static final double kFlywheelMaxRPM = 5000;
        public static final double kBonnetMaxExtensionMM = 50; // Change me!

        public static final double[] kFlywheelPID = {0.005, 0, 0.001};
        public static final double[] kBonnetPID = {0.005, 0, 0}; // Tune me!


        // Hopper Constants
        public static final int kBeltCANID = 7; // Change me!
        public static final int kCorralCANID = 8; // Change me!
        public static final int kKickerCANID = 9; // Change me!

        public static final boolean kBeltInverted = false; // May need changed!
        public static final boolean kCorralInverted = false; // May need changed!
        public static final boolean kKickerInverted = false; // May need changed!

        public static final double kKickerConversionFactor = 0; // Change me!

        public static final int kKickerMotorMaxRPM = 500; // Change Me!
        public static final double[] kKickerPID = {0.005, 0, 0}; // Tune me!


        // Intake Constants
        public static final int kintakeExtensionCANID = 10; // Change me!
        public static final int kRollerCANID = 11; // Change me!

        public static final boolean kintakeExtensionInverted = false; // May need changed!
        public static final boolean kRollerInverted = false; // May need changed!

        public static final double kintakeExtensionConversionFactor = 0; // Change me!
        public static final int kintakeExtensionMaxExtensionMM = 50; // Change me!

        public static final double[] kintakeExtensionPID = {0.005, 0, 0}; // Tune me!


        // Climber Constants
        public static final int kClimberExtensionCANID = 12; // Change me!
        public static final int kClimberCANID = 13; // Change me!
        public static final int kLatchChannel = 14;

        public static final boolean kClimberExtensionInverted = false; // May need changed!
        public static final boolean kClimberInverted = false; // May need changed!

        public static final double kClimberExtensionConversionFactor = 0; // Change me!
        public static final double kClimberConversionFactor = 0; // Change me!

        public static final int kClimberExtensionMaxExtensionMM = 50; // Change me!
        public static final int kClimberMaxExtensionMM = 50;

        public static final double[] kClimberExtensionPID = {0.005, 0, 0}; // Tune me!
        public static final double[] kClimberPID = {0.005, 0, 0,}; // Tune me!

        public static final int kLatchLockPosition = 50; // Change me!
        public static final int kLatchUnlockPosition = 0; // May need changed!
    }
}
