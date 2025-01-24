package org.pihisamurai.frc2025.robot.commands.drive;

import java.util.List;

import org.pihisamurai.frc2025.robot.Constants.DriveConstants.PathPlannerConstants;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** This automatically drives to a pose without using A* to generate a trajectory, useful for when we know there are no obstructions on the field between the robot and the desired pose */
public class DirectDriveToPoseCommand extends Command {

    private  Command followPathCommand;

    public DirectDriveToPoseCommand(DriveSubsystem drive,Pose2d targetPose) {
        Pose2d currentPose = drive.getPose();
        Pose2d intermediatePose = currentPose.transformBy(targetPose.minus(currentPose).times(0.5));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            intermediatePose,
            targetPose
        );
        PathPlannerPath path = new PathPlannerPath(waypoints, PathPlannerConstants.pathConstraints, null, new GoalEndState(0, targetPose.getRotation()));
        path.preventFlipping = true;
        followPathCommand = AutoBuilder.followPath(path);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        followPathCommand.initialize();
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