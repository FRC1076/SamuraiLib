// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.commands.drive.DirectDriveToPoseCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.utils.Localization;
import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotOffset;

import lib.utils.GeometryUtils;
import lib.vision.VisionLocalizationSystem;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged rearLeftInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged rearRightInputs = new ModuleIOInputsAutoLogged();
    private Boolean hasSetAlliance = false; // Wait until the driverstation had an alliance before setting it
    public final DriveCommandFactory CommandBuilder;
    private final VisionLocalizationSystem vision;

    public DriveSubsystem(DriveIO io, VisionLocalizationSystem vision) {
        this.io = io;
        this.vision = vision;
        vision.registerMeasurementConsumer(this.io::addVisionMeasurement); // In DriveIOHardware, addVisionMeasurement is built into the SwerveDrivetrain class
        try {
            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                () -> driveInputs.Speeds,
                (speeds,feedforwards) -> driveCO(speeds),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(5, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(5, 0, 0)
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
        CommandBuilder = new DriveCommandFactory(this);

        
    }

    @Override
    public void periodic(){
        // updateModuleInputs and processInputs are only used for logging
        io.periodic(); //currently just for calling sim
        vision.update();
        io.updateInputs(driveInputs);
        // io.updateModuleInputs(frontLeftInputs,0);
        // io.updateModuleInputs(frontRightInputs,1);
        // io.updateModuleInputs(rearLeftInputs,2);
        // io.updateModuleInputs(rearRightInputs,3);
        Logger.processInputs("Drive", driveInputs);
        // Logger.processInputs("Drive/FrontLeft",frontLeftInputs);
        // Logger.processInputs("Drive/FrontRight",frontRightInputs);
        // Logger.processInputs("Drive/RearLeft",rearLeftInputs);
        // Logger.processInputs("Drive/RearRight",rearRightInputs);

        if (DriverStation.getAlliance().isPresent() && !hasSetAlliance) {
            hasSetAlliance = true;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                io.setAllianceRotation(Rotation2d.fromDegrees(180));
            } else {
                io.setAllianceRotation(Rotation2d.fromDegrees(0));
            }
        }
    }

    /** Swerve drive request with chassis-oriented chassisSpeeds */
    public void driveCO(ChassisSpeeds speeds) {
        io.acceptRequest(new ApplyRobotSpeeds().withSpeeds(speeds));
    }

    /** Swerve drive request with field-oriented chassisSpeeds */
    public void driveFO(ChassisSpeeds speeds) {
        io.acceptRequest(new ApplyFieldSpeeds().withSpeeds(speeds).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
    }

    /** Swerve drive request with heading lock */
    public void driveFOHeadingLocked(double xMetersPerSecond, double yMetersPerSecond, Rotation2d targetDirection) {
        FieldCentricFacingAngle request = new FieldCentricFacingAngle()
        .withVelocityX(xMetersPerSecond)
        .withVelocityY(yMetersPerSecond)
        .withTargetDirection(targetDirection)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        request.HeadingController.setPID(3.5, 0, 0);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        io.acceptRequest(request);
    }

    /** Set swerveRequest */
    public void acceptRequest(SwerveRequest request) {
        io.acceptRequest(request);
    }

    /** Resets the pose of the robot */
    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Makes the current heading of the robot the default zero degree heading
     * (Used if forward is the wrong direction)
     */
    public void resetHeading() {
        io.resetHeading();
    }

    public Rotation2d getHeading() {
        return io.getPose().getRotation();
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return io.getPose();
    }

    public class DriveCommandFactory {
        private final DriveSubsystem drive;
        private final Map<ReefFace,Command> leftBranchAlignmentCommands = new HashMap<>();
        private final Map<ReefFace,Command> reefCenterAlignmentCommands = new HashMap<>();
        private final Map<ReefFace,Command> rightBranchAlignmentCommands = new HashMap<>();
        private DriveCommandFactory(DriveSubsystem drive) {
            this.drive = drive;
            for (ReefFace face : ReefFace.values()) {
                leftBranchAlignmentCommands.put(face,directDriveToPose(GeometryUtils.rotatePose(face.leftBranch.transformBy(robotOffset),Rotation2d.k180deg)));
                reefCenterAlignmentCommands.put(face,directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotOffset),Rotation2d.k180deg)));
                rightBranchAlignmentCommands.put(face,directDriveToPose(GeometryUtils.rotatePose(face.rightBranch.transformBy(robotOffset),Rotation2d.k180deg)));
            }
        }

        public Command pathfindToPose(Pose2d targetPose) {
            return AutoBuilder.pathfindToPose(
                targetPose,
                PathPlannerConstants.pathConstraints,
                0.0
            );
        }
    
        public Command followPath(PathPlannerPath path){
            return AutoBuilder.followPath(path);
        }

        public TeleopDriveCommand teleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
            return new TeleopDriveCommand(drive,xSupplier,ySupplier,omegaSupplier);
        }

        public Command directDriveToPose(Pose2d targetPose) {
            return new DirectDriveToPoseCommand(drive, targetPose);
        }

        public Command directDriveToNearestLeftBranch() {
            return new SelectCommand<>(leftBranchAlignmentCommands,() -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestReefFace() {
            return new SelectCommand<>(reefCenterAlignmentCommands,() -> Localization.getClosestReefFace(drive.getPose()));
        }

        public Command directDriveToNearestRightBranch() {
            return new SelectCommand<>(rightBranchAlignmentCommands,() -> Localization.getClosestReefFace(drive.getPose()));
        }
        
        public Command applySwerveRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> acceptRequest(requestSupplier.get()));
        }

    }

}
