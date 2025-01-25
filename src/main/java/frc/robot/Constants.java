// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Arrays;

import static edu.wpi.first.units.Units.Rotation;

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

    public static class IndexConstants {
        public static final int kLeadMotorPort = -10;
        public static final int kFollowMotorPort = -10;

        public static final double kCurrentLimit = 40;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;
    }
}
