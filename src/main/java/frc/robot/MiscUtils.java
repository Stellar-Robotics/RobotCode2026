// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MiscUtils {

    // CLAMP functionality - Yay!
    public static double clamp(double min, double max, double value) {
        return Math.min(Math.max(min, value), max);
    }

    // Translate to non-linear response curve
    public static double transformRange(double linearInput, double exp) {
        return Math.abs(linearInput) * linearInput;
    }

    // Circular Deadband
    public static double[] circularDeadband(double xRadius, double yRadius, double deadbandRadius) {

        // Calculate magnitude formed for X and Y inputs
        double vectorMagnitude = Math.sqrt(Math.pow(xRadius, 2) + Math.pow(yRadius, 2));

        if (vectorMagnitude > deadbandRadius) {
            // When the vector is outside the deadband radius

            // Normalize and Apply a deadband scale to ensure a soft transition
            // from the deadband to the active zone
            double xValueScaled = (vectorMagnitude - deadbandRadius) * ( xRadius / vectorMagnitude) / (1 - deadbandRadius);
            double yValueScaled = (vectorMagnitude - deadbandRadius) * ( yRadius / vectorMagnitude) / (1 - deadbandRadius);

            // Clamp the scaled values to ensure they
            // cannot overreach
            double xValueClamped = clamp(-1, 1, xValueScaled);
            double yValueClamped = clamp(-1, 1, yValueScaled);

            // Send sime debugging info to SmartDashboard
            SmartDashboard.putBoolean("Is Active Zone", true);

            SmartDashboard.putNumber("Analog X", xRadius);
            SmartDashboard.putNumber("Analog Y", yRadius);

            SmartDashboard.putNumber("X Processed", xValueClamped);
            SmartDashboard.putNumber("Y Processed", yValueClamped);

            // Return the data in the form of an array
            double[] adjustedValues = {xValueClamped, yValueClamped};
            return adjustedValues;

        } else {

            // Send sime debugging info to SmartDashboard
            SmartDashboard.putBoolean("Is Active Zone", false);
        
            SmartDashboard.putNumber("Analog X", xRadius);
            SmartDashboard.putNumber("Analog Y", yRadius);

            SmartDashboard.putNumber("X Processed", 0);
            SmartDashboard.putNumber("Y Processed", 0);

            // Return 0s if the magnitude is within the db
            double[] adjustedValues = {0, 0};
            return adjustedValues;
        }
    }

    // A quick function to get red aliance status
    public static BooleanSupplier isRedAlliance() {
        // Return an anonymous function for the caller to run when needed
        return () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }

    // A function to normaize the PID setpoint in order to eliminate osscilations along -180 and 180
    public static double normalizeAngle(double angle) {
        if (angle > 180) {
            return angle - 360;
        } else if (angle < -180) {
            return angle + 360;
        } else {
            return angle;
        }
    }

    // A function that will incrament a value every loop of the code, creating a ramp
    public static void ramp(double incrament, double cap, Consumer<Double> operation) {
        // DONT USE, CURRENTLY CAUSES STACK OVERFLOW
        if (incrament < cap) {
            operation.accept(incrament);
            ramp(incrament, cap, operation);
        } else {
            return;
        }

    }

    // Averages two poses
    public static Pose2d averageTwoPoses(Pose2d pose1, Pose2d pose2) {
        Translation2d translation1 = pose1.getTranslation();
        Translation2d translation2 = pose2.getTranslation();

        Translation2d averageTranslation = new Translation2d(
            (translation1.getX() + translation2.getX()) / 2,
            (translation1.getY() + translation2.getY()) / 2
        );

        Rotation2d rotation1 = pose1.getRotation();
        Rotation2d rotation2 = pose2.getRotation();

        Rotation2d averageRotation = new Rotation2d(
            (rotation1.getRadians() + rotation2.getRadians()) / 2
        );

        Pose2d averagePose = new Pose2d(averageTranslation, averageRotation);
        return averagePose;
    }
}
