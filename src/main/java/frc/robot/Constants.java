// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Arrays;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.path.PathConstraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class Akit {
        public static final int currentMode = 1;
    }
    
    public static class DriveConstants{
        public static class DriverControlConstants{
            public static final double singleClutchTranslationFactor = 0.5;
            public static final double singleClutchRotationFactor = 0.5;
            public static final double doubleClutchTranslationFactor = 0.3;
            public static final double doubleClutchRotationFactor = 0.35;
        }

        public static class PathPlannerConstants{
            public static final PathConstraints pathConstraints = new PathConstraints(4.69, 25, Units.degreesToRadians(1080), Units.degreesToRadians(1080));
        }

        public enum HeadingMode{
            NORMAL,
            POINT_TO_REEF,
            LEFT_CORAL_STATION,
            RIGHT_CORAL_STATION,
            STRAIGHT,
        }

        public enum ClutchMode{
            NORMAL,
            SINGLE_CLUTCH,
            DOUBLE_CLUTCH,
        }
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

        public static final List<Pose2d> leftBranchCoordinates = Arrays.asList(
            new Pose2d(3.93, 3.39, Rotation2d.fromDegrees(-120.00)),
            new Pose2d(3.66, 4.19, Rotation2d.fromDegrees(180.00)),
            new Pose2d(4.21, 4.83, Rotation2d.fromDegrees(120.00)),
            new Pose2d(5.04, 4.67, Rotation2d.fromDegrees(60.00)),
            new Pose2d(5.32, 3.86, Rotation2d.fromDegrees(0.00)),
            new Pose2d(4.77, 3.23, Rotation2d.fromDegrees(-60.00)),
            new Pose2d(13.33, 3.23, Rotation2d.fromDegrees(-60.00)),
            new Pose2d(13.89, 3.86, Rotation2d.fromDegrees(0.00)),
            new Pose2d(13.61, 4.66, Rotation2d.fromDegrees(60.00)),
            new Pose2d(12.78, 4.83, Rotation2d.fromDegrees(120.00)),
            new Pose2d(12.23, 4.19, Rotation2d.fromDegrees(180.00)),
            new Pose2d(12.50, 3.39, Rotation2d.fromDegrees(-120.00))
        );

        public static final List<Pose2d> rightBranchCoordinates = Arrays.asList(
            new Pose2d(4.21, 3.23, Rotation2d.fromDegrees(-120.00)),
            new Pose2d(3.66, 3.86, Rotation2d.fromDegrees(180.00)),
            new Pose2d(3.93, 4.66, Rotation2d.fromDegrees(120.00)),
            new Pose2d(4.77, 4.83, Rotation2d.fromDegrees(60.00)),
            new Pose2d(5.32, 4.19, Rotation2d.fromDegrees(0.00)),
            new Pose2d(5.04, 3.39, Rotation2d.fromDegrees(-60.00)),
            new Pose2d(13.61, 3.39, Rotation2d.fromDegrees(-60.00)),
            new Pose2d(13.89, 4.19, Rotation2d.fromDegrees(0.00)),
            new Pose2d(13.33, 4.83, Rotation2d.fromDegrees(60.00)),
            new Pose2d(12.50, 4.66, Rotation2d.fromDegrees(120.00)),
            new Pose2d(12.23, 3.86, Rotation2d.fromDegrees(180.00)),
            new Pose2d(12.78, 3.23, Rotation2d.fromDegrees(-120.00))
        );
        
        public static final Translation2d blueReefCenter = new Translation2d(4.487, 4.010);
        public static final Translation2d redReefCenter = new Translation2d(13.062, 4.010);

        public static final Rotation2d leftCoralStationAngle = Rotation2d.fromDegrees(-55);
        public static final Rotation2d rightCoralStationAngle = Rotation2d.fromDegrees(55);
    }
}
