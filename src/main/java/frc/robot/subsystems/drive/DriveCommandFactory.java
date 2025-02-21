package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import frc.robot.Constants.FieldConstants.ReefFace;
import frc.robot.commands.drive.DirectDriveToPoseCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.Localization;
import lib.utils.GeometryUtils;

import static frc.robot.Constants.DriveConstants.PathPlannerConstants.robotOffset;

public class DriveCommandFactory {
    private final DriveSubsystem drive;
    private final VisionSubsystem vision;
    private final Map<ReefFace,Command> leftBranchAlignmentCommands = new HashMap<>();
    private final Map<ReefFace,Command> reefCenterAlignmentCommands = new HashMap<>();
    private final Map<ReefFace,Command> rightBranchAlignmentCommands = new HashMap<>();
    public DriveCommandFactory(DriveSubsystem drive,VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        
        for (ReefFace face : ReefFace.values()) {
            leftBranchAlignmentCommands.put(face,
                Commands.sequence(
                    Commands.runOnce(() -> vision.enableRearCameras(false)),
                    directDriveToPose(GeometryUtils.rotatePose(face.leftBranch.transformBy(robotOffset),Rotation2d.k180deg)),
                    Commands.runOnce(() -> vision.enableRearCameras(true))
                )
            );
            reefCenterAlignmentCommands.put(face,
                Commands.sequence(
                    Commands.runOnce(() -> vision.enableRearCameras(false)),
                    directDriveToPose(GeometryUtils.rotatePose(face.AprilTag.transformBy(robotOffset),Rotation2d.k180deg)),
                    Commands.runOnce(() -> vision.enableRearCameras(true))
                )
            );
            rightBranchAlignmentCommands.put(face,
                Commands.sequence(
                    Commands.runOnce(() -> vision.enableRearCameras(false)),
                    directDriveToPose(GeometryUtils.rotatePose(face.rightBranch.transformBy(robotOffset),Rotation2d.k180deg)),
                    Commands.runOnce(() -> vision.enableRearCameras(true))
                )
            );
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
        return Commands.run(() -> drive.acceptRequest(requestSupplier.get()),drive);
    }
}
