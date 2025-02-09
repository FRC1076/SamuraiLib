// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

import org.apache.commons.lang3.NotImplementedException;

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
        public static final int kOperatorControllerPort = 1;
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

        public static class PathPlannerConstants {
            public static final PathConstraints pathConstraints = new PathConstraints(4.69, 25, Units.degreesToRadians(1080), Units.degreesToRadians(1080));
            public static final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.kZero);
            public static final double pathGenerationToleranceMeters = 0.011; // technically it's anything larger than 0.01, but I'm adding .001 just to be safe

            public static class Control {
                public static final PIDConstants transPID = new PIDConstants(5,0,0);
                public static final PIDConstants rotPID = new PIDConstants(5,0,0);
            }
        }
    }

    public static class SuperstructureConstants {

        //Grabber Possession State
        public enum GrabberPossession {
            EMPTY(0,2.8605),
            CORAL(0,2.8605),
            ALGAE(0,2.8605);

            public final double wrist_kG;
            public final double elevator_kG;

            private GrabberPossession(double wrist_kG, double elevator_kG) {
                this.wrist_kG = wrist_kG;
                this.elevator_kG = elevator_kG;
            }
        }

        //Index Possession State
        public enum IndexPossession {
            EMPTY,
            CORAL
        }

        //Grabber State
        public enum GrabberState {

            IDLE(0,0),
            
            ALGAE_INTAKE(-12,-12),
            CORAL_INTAKE(5,5),

            ALGAE_OUTTAKE(6,6),
            CORAL_OUTTAKE(12,12),
            DEFAULT_OUTTAKE(12,12);


            public final double leftVoltage;
            public final double rightVoltage;

            private GrabberState(double leftVoltage, double rightVoltage) {
                this.leftVoltage = leftVoltage;
                this.rightVoltage = rightVoltage;
            }
        }

        //Index State
        public enum IndexState {
            EMPTY_IDLE(false),
            CORAL_INTAKE(true),
            CORAL_TRANSFER(true),
            CORAL_IDLE(false);
            public final boolean running; //Whether or not the indexer motors are running
            private IndexState(boolean running) {
                this.running = running;
            }
        }

        //Represent the elevator height and wrist angle for different positions, the full position of the grabber
        //Should we have an eject state with an optional elevator height? just to immediately eject if a game piece is stuck
        public enum WristevatorPreset {
            
            TRAVEL(0.08128,90),
            ALGAE_TRAVEL(0.08128, 65),

            CORAL_TRANSFER(0.08128,-23.5), //same as CORAL_DIRECT_INTAKE

            L1(-1,-1), //Placeholder
            L2(0.71628,-35),
            L3(1.11252,-35),
            L4(1.8161,-45),

            GROUND_INTAKE(-1, -1),
            LOW_INTAKE(0.9144,-35),
            HIGH_INTAKE(1.30556,-35),

            PROCESSOR(0.184277,0),
            NET(1.8288,65);

            public final double elevatorHeightMeters;
            public final Rotation2d wristAngle;
            
            private WristevatorPreset(double elevatorHeightMeters, double wristAngleDegrees) {
                this.elevatorHeightMeters = elevatorHeightMeters;
                this.wristAngle = Rotation2d.fromDegrees(wristAngleDegrees);
            }
        }
    }

    /** Contains data about the field */
    public static class FieldConstants {
        private static final double branchOffset = Units.inchesToMeters(6.469);
        private static final Transform2d leftBranchTransform = new Transform2d(0.0, -branchOffset, Rotation2d.kZero);
        private static final Transform2d rightBranchTransform = new Transform2d(0.0, branchOffset, Rotation2d.kZero);
        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {
            // IMPORTANT: Fudge factors are always positive and should be in meters (use the Units.inchesToMeters() method)

            //Blue Reef
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, null, null),
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, null, null),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, null, null),
            BLU_REEF_GH(21, 5.321046, 4.025900, 0.0, null, null),
            BLU_REEF_IJ(20, 4.904740, 4.745482, 60.0, null, null),
            BLU_REEF_KL(19, 4.073906, 4.745482, 120.0, null, null),

            //Red Reef
            RED_REEF_AB(7, 13.890498, 4.025900, 0.0, null, null),
            RED_REEF_CD(8, 13.474446, 4.745482, 60., null, null),
            RED_REEF_EF(9, 12.643358, 4.745482, 120.0, null, null),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, null, null),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, null, null),
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, null, null);


            public final Double leftBranchFudgeTransform;
            public final Double rightBranchFudgeTransform;
            public final Pose2d leftBranch;
            public final Pose2d rightBranch;
            public final Pose2d AprilTag;
            public final int AprilTagID;

            //AT stands for AprilTag
            private ReefFace(int AprilTagID, double AT_x, double AT_y, double AT_theta, Double leftBranchFudgeTransform, Double rightBranchFudgeTransform) {
                this.AprilTagID = AprilTagID;
                this.AprilTag = new Pose2d(AT_x, AT_y, Rotation2d.fromDegrees(AT_theta));
                this.leftBranchFudgeTransform = leftBranchFudgeTransform;
                this.rightBranchFudgeTransform = rightBranchFudgeTransform;

                if(this.leftBranchFudgeTransform == null) {
                    this.leftBranch = AprilTag.transformBy(leftBranchTransform);
                } else {
                    this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
                }
                
                if(this.rightBranchFudgeTransform == null) {
                    this.rightBranch = AprilTag.transformBy(rightBranchTransform);
                } else {
                    this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
                }
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
            BLU_PROCESSOR(5.973318,-0.00381,90), //Taken from April Tag coordinates
            RED_PROCESSOR(11.56081,	8.05561,	270), //Taken from April Tag coordinates
            BLU_RIGHT_STATION(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),55),
            BLU_LEFT_STATION(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),305),
            RED_RIGHT_STATION(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),-125),
            RED_LEFT_STATION(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),125);

            public final Pose2d pose;

            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) {
                this.pose = new Pose2d(xMeters,yMeters,Rotation2d.fromDegrees(omegaDeg));
            }
        }
    }

    public static class ElevatorConstants {
        public static final int kMotorPort0 = -1;
        public static final int kMotorPort1 = -1;
        
        public static final double elevatorPositionToleranceMeters = Units.inchesToMeters(0.5);
        public static final double maxOperatorControlVolts = 6;

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
            public static final double kCurrentLimit = 60;
        }


        public static class Control {
            //PID Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kVelP = 0.9;
            public static final double kVelI = 0;
            public static final double kVelD = 0;
            
            public static final double kPosP = 0.8;
            public static final double kPosI = 0.0;
            public static final double kPosD = 0.0;

            //Feedforward Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kS = 0; //Static gain (voltage)
            public static final double kG = 2.8605; //Gravity gain (voltage)
            public static final double kV = 0; // velocity game
            public static final double kA = 0; //Acceleration Gain
        }
    }

    public static class ElevatorSimConstants {
        //RANDOM ports
        public static final int kSimMotorPort0 = 20;
        public static final int kSimMotorPort1 = 21;
        
        public static final double kElevatorGearing = 60.0/11.0;
        public static final double kCarriageMass = Units.lbsToKilograms(30); //kg
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0); //taken from example, do not know what this is
        public static final double kMinElevatorHeightMeters = 0;
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72.0);

        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;

        public static class Control {
            //PID Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kP = 6;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            //Feedforward Constants - STILL SET TO WAPUR ELEVATOR VALUES
            public static final double kS = 0; // Static gain (voltage)
            public static final double kG = 2.8605; // Gravity gain (voltage)
            public static final double kV = 0; // velocity gain
            public static final double kA = 0; // Acceleration Gain
        }
    }

    public static final class GrabberConstants {
        public static final int kLeftMotorPort = -1;
        public static final int kRightMotorPort = -1;
        
        public static final double kCurrentLimit = 40; 
    }

    public static class WristConstants {
        public static final int kLeadMotorPort = -2;
        public static final int kFollowMotorPort = -3;

        public static final double wristAngleToleranceRadians = Units.degreesToRadians(1);
        public static final double maxOperatorControlVolts = 6;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;

        // source: https://docs.revrobotics.com/brushless/spark-max/encoders/alternate-encoder
        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2*Math.PI; // rotations to radians
        public static final double kVelocityConversionFactor = (2*Math.PI) / 60.0; // rpm to radians/second

        public static final class Control {

            public static final double kVelP = 0.2;
            public static final double kVelI = 0;
            public static final double kVelD = 0;

            // PID constants
            public static final double kPosP = 0.4;
            public static final double kPosI = 0.0;
            public static final double kPosD = 0.0;

            // feed forward constants
            public static final double kS = 0.0; // static gain in volts
            public static final double kG = 0.0; // gravity gain in volts
            public static final double kV = 0.0; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared
        }
    }

    public static class WristSimConstants {
        // values are NOT CORRECT
        public static final double kWristGearingReductions = 125;
        public static final double kWristLength = Units.feetToMeters(1); // excludes a 5 inch fixed piece
        public static final double kWristMass = 2;
        public static final double kMinAngleRads = -0.75 * Math.PI;
        public static final double kMaxAngleRads = 0.75 * Math.PI;

        public static class SimControl {
            public static final double kP = 1.8;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0.0; // static gain in volts
            public static final double kG = 0.0553; // gravity gain in volts
            public static final double kV = 0.0; // velocity gain in volts per radian per second
            public static final double kA = 0.0; // acceleration gain in volts per radian per second squared
        }
    }

    public static class IndexConstants {
        public static final int kLeadMotorPort = -10;
        public static final int kFollowMotorPort = -10;

        public static final double kCurrentLimit = 40;
        public static final double kIndexVoltage = 6;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = false;
    }

    public static class BeamBreakConstants{
        public static final int indexBeamBreakPort = 0;
        public static final int transferBeamBreakPort = 1;
        public static final int grabberBeamBreakPort = 2;
    }

    public static class WristevatorConstants {
        public static class Control {
            //POSITION CONTROL
            public static final double wrist_kP = 6;
            public static final double wrist_kI = 0;
            public static final double wrist_kD = 0;
            
            public static final double elevator_kP = 1;
            public static final double elevator_kI = 0.3;
            public static final double elevator_kD = 0.1;

            public static final double wristVelMOE = 0.7;
            public static final double wristPosMOE = 0.1;
            public static final double elvtrVelMOE = 7;
            public static final double elvtrPosMOE = 0.1;
        }

        public static class TrajectoryTests {
            public static final List<WristevatorState> testTraj1 = new ArrayList<>();
            static {
                testTraj1.add(new WristevatorState(0.1, Rotation2d.kZero, 0, 0));
                testTraj1.add(new WristevatorState(0.5, Rotation2d.fromDegrees(-90), 0.2, 0));
                testTraj1.add(new WristevatorState(1.8, Rotation2d.fromDegrees(-60), 0, 0));
            }
        }
    }

    public static class VisionConstants {
        public static class PVConstants {
            public static enum CamConfig {
                FL_ELEVATOR_CAM("FL_ELEVATOR_CAM",7.286,2.794,15.482,11.385,17.961,40),
                FR_ELEVATOR_CAM("FR_ELEVATOR_CAM",0,0,0,0,0,0);

                public final String name;
                public final Transform3d offset;

                //Rotations are represented as quaternions, as they are invariant between coordinate systems and reference frames
                private CamConfig(String name, double xInches, double yInches, double zInches, double rollDegrees, double pitchDegrees, double yawDegrees) {
                    this.name = name;
                    this.offset = new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(xInches),
                            Units.inchesToMeters(yInches),
                            Units.inchesToMeters(zInches)
                        ), 
                        new Rotation3d(
                            Units.degreesToRadians(rollDegrees),
                            Units.degreesToRadians(pitchDegrees),
                            Units.degreesToRadians(yawDegrees)
                        )
                    );
                }
            }
        }
    }

    private Constants() {
        throw new NotImplementedException();
    }
}