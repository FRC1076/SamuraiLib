// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class Akit {
        public static final int currentMode = 1;
    }
    
    public static class DriveConstants{
        public static final double singleClutchTranslationFactor = 0.5;
        public static final double singleClutchRotationFactor = 0.5;
        public static final double doubleClutchTranslationFactor = 0.3;
        public static final double doubleClutchRotationFactor = 0.35;
    }

    public static class Coordinates{
        public static final List<Pose2d> reefAprilCoordinates = Arrays.asList(
            new Pose2d(4.073906, 3.306318, Rotation2d.fromDegrees(240)), // 17
            new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(180)), //18
            new Pose2d(4.073906, 4.745482, Rotation2d.fromDegrees(120)), //19
            new Pose2d(4.90474, 4.74582, Rotation2d.fromDegrees(60)), //20
            new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(0)), //21
            new Pose2d(4.90474, 3.306318, Rotation2d.fromDegrees(300)), // 22 - end of blue
            new Pose2d(13.474446, 3.306318, Rotation2d.fromDegrees(300)), //6
            new Pose2d(13.890498, 4.0259, Rotation2d.fromDegrees(0)), //7
            new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(60)), //8
            new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(120)), //9
            new Pose2d(12.227306, 4.0259, Rotation2d.fromDegrees(180)), //10
            new Pose2d(12.643358, 3.306318, Rotation2d.fromDegrees(240)) //11
        );
    }

    public static class ElevatorConstants {
        public static final int kMotorPort0 = -1;
        public static final int kMotorPort1 = -1;

        public static final boolean leadMotorInverted = false;
        public static final boolean followMotorInverted = false;

        //Heights measured in meters
        public static final double lowHeight = 0;
        public static final double autonHeight = 0;
        public static final double midHeight = 0;
        public static final double highHeight = 0;

        //public static final double minHeightMeters = 0;
        //public static final double maxHeightMeters = 0.85; //Temporary

        //https://wcproducts.com/collections/gearboxes/products/wcp-single-stage-gearbox  Inches.of(0.25).in(Meters)
        // still set to WAPUR elevator units, need to be changed
        public static final double kVelocityConversionFactor = (11/60.0) * 22 * 0.00635 / 60.0; //Gear ratio & chain pitch & rpm -> m/s
        public static final double kPositionConversionFactor = (11/60.0) * 22 * 0.00635; //Gear ratio & chain pitch
        public static class Electrical {
            public static final double kVoltageCompensation = 12;
            public static final double kCurrentLimit = 40;
        }


        public static class Control {
            //PID Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kP = 8;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            //Feedforward Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kS = 0; //Static gain (voltage)
            public static final double kG = 0.6; //Gravity gain (voltage)
            public static final double kV = 12; // velocity game
            public static final double kA = 0; //Acceleration Gain
        }
    }
    public static final class GrabberConstants {
        public static final int kLeftMotorPort = -1;
        public static final int kRightMotorPort = -1;
        
        public static final double kCurrentLimit = 40; 
    }
    public static class WristConstants {
        public static final int kLeadMotorPort = -1;
        public static final int kFollowMotorPort = -1;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;

        // source: https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2*Math.PI; // rotations to radians
        public static final double kVelocityConversionFactor = (2*Math.PI) / 60.0; // rpm to radians/second

        public static final class Control {
            // PID constants
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            // feed forward constants
            public static final double kS = 0.0; // static gain in volts
            public static final double kG = 0.0; // gravity gain in volts
            public static final double kV = 0.0; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared
        }
    }

    public static class IndexConstants {
        public static final int kLeadMotorPort = -10;
        public static final int kFollowMotorPort = -10;

        public static final double kCurrentLimit = 40;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;
    }
}
