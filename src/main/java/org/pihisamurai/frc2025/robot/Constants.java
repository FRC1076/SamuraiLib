// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pihisamurai.frc2025.robot;

import org.pihisamurai.lib.control.LQRHolonomicDriveController.LQRHolonomicDriveControllerTolerances;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    
    public static class DriveConstants {

        public static class DriverControlConstants {
            public static final double singleClutchTranslationFactor = 0.5;
            public static final double singleClutchRotationFactor = 0.5;
            public static final double doubleClutchTranslationFactor = 0.3;
            public static final double doubleClutchRotationFactor = 0.35;
            public static final double maxTranslationSpeedMPS = 5;
            public static final double maxRotationSpeedRadPerSec = 5;
        }

        public static class SystemControlConstants {
            public static final LQRHolonomicDriveControllerTolerances driveControlWeights = new LQRHolonomicDriveControllerTolerances(0.4, 1.0, 0.1, 3);
        }

        public static class PathPlannerConstants {
            public static final PathConstraints pathConstraints = new PathConstraints(4.69, 25, Units.degreesToRadians(1080), Units.degreesToRadians(1080));
            public static final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.kZero);
        }

    }

    /** Contains data about the field */
    public static class FieldConstants {
        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {

            //Blue Reef
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, 3.930, 3.400, 4.210, 3.200),
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, 3.658, 4.180, 3.658, 3.860),
            BLU_REEF_LK(19, 4.073906, 4.745482, 120.0, 4.190, 4.850, 3.920, 4.670),
            BLU_REEF_JI(20, 4.904740, 4.745482, 60.0, 5.060, 4.690, 4.780, 4.840),
            BLU_REEF_HG(21, 5.321046, 4.025900, 0.0, 5.321, 3.850, 5.321, 4.170),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, 4.780, 3.220, 5.060, 3.340),

            //Red Reef
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, 15.350, 3.210, 13.640, 3.340),
            RED_REEF_BA(7, 13.890498, 4.025900, 0.0, 13.910, 3.860, 13.910, 4.190),
            RED_REEF_DC(8, 13.474446, 4.745482, 60.0, 13.640, 4.690, 13.360, 4.830),
            RED_REEF_FE(9, 12.643358, 4.745482, 120.0, 12.780, 4.830, 12.510, 4.680),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, 12.220, 4.190, 12.220, 3.850),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, 12.490, 3.360, 12.790, 3.210);


            public final Pose2d leftBranch;
            public final Pose2d rightBranch;
            public final Pose2d AprilTag;
            public final int AprilTagID;

            private ReefFace(int AprilTagID, double AT_x, double AT_y, double AT_theta, double L_x, double L_y, double R_x, double R_y) {
                this.AprilTagID = AprilTagID;
                this.AprilTag = new Pose2d(AT_x, AT_y, Rotation2d.fromDegrees(AT_theta));
                this.leftBranch = new Pose2d(L_x, L_y,Rotation2d.fromDegrees(AT_theta));
                this.rightBranch = new Pose2d(R_x, R_y,Rotation2d.fromDegrees(AT_theta));
            }
        }

        //Generic rotation-agnostic points of interest
        public enum PointOfInterest {
            BLU_REEF(4.487,4.010),
            RED_REEF(13.062,4.010);

            public final Translation2d position;
            private PointOfInterest(double xMeters, double yMeters) {
                this.position = new Translation2d(xMeters,yMeters);
            }
        }

        //Poses of interest
        public enum PoseOfInterest {
            BLU_PROCESSOR(0,0,0), //Placeholder
            RED_PROCESSOR(0,0,0), //Placeholder
            BLU_CORAL_STATION_PROCESSOR(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),54),
            BLU_CORAL_STATION_OPPOSITE(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),306),
            RED_CORAL_STATION_PROCESSOR(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),-125),
            RED_CORAL_STATION_OPPOSITE(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),125);

            public final Pose2d pose;
            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) {
                this.pose = new Pose2d(xMeters,yMeters,Rotation2d.fromDegrees(omegaDeg));
            }
        }
    }

}
