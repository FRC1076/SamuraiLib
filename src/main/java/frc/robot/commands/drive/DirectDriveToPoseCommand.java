package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import frc.robot.subsystems.drive.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants.PathPlannerConstants;
import lib.utils.GeometryUtils;

/** This automatically drives to a pose without using A* to generate a trajectory, useful for when we know there are no obstructions on the field between the robot and the desired pose */
public class DirectDriveToPoseCommand extends Command {

    private Command followPathCommand;
    private final Pose2d targetPose;
    private final DriveSubsystem m_drive;

    public DirectDriveToPoseCommand(DriveSubsystem drive,Pose2d targetPose) {
        this.m_drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drive.getPose();
        Pose2d startingWaypoint = new Pose2d(currentPose.getTranslation(), GeometryUtils.angleToPose(currentPose, targetPose));
        //Pose2d intermediatePose = currentPose.transformBy(targetPose.minus(currentPose).times(0.5));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startingWaypoint,
            targetPose
        );
        if(targetPose.getTranslation().getDistance(startingWaypoint.getTranslation()) > PathPlannerConstants.pathGenerationToleranceMeters){
            PathPlannerPath path = new PathPlannerPath(waypoints, PathPlannerConstants.pathConstraints, null, new GoalEndState(0, targetPose.getRotation()));
            path.preventFlipping = true;
            followPathCommand = AutoBuilder.followPath(path);
        }
        else{
            followPathCommand = Commands.none();
        }
        followPathCommand.schedule();
    }

    @Override
    public void execute(){
        followPathCommand.execute();
    }
    
    @Override
    public boolean isFinished() {
        return followPathCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        followPathCommand.end(interrupted);
    }

}