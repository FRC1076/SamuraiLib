// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pihisamurai.frc2025.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.List;

import org.pihisamurai.frc2025.robot.Constants.Coordinates;
import org.pihisamurai.frc2025.robot.Constants.Coordinates.ReefAlignment;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.PathPlannerConstants;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.fasterxml.jackson.core.TreeNode;
import com.pathplanner.lib.path.GoalEndState;


/** An example command that uses an example subsystem. */
public class DriveToReef extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final ReefAlignment reefAlignment;
  private Pose2d nearestBranch;
  private Command pathfindCommand = Commands.none();
  private final Transform2d robotOffset = new Transform2d(0.4572, 0, Rotation2d.fromDegrees(0));
  private Pose2d startWaypointPose;
  private Pose2d endWaypointPose;

   /**
    * Creates a new ExampleCommand.
    *
    * @param subsystem The subsystem used by this command.
    */
    public DriveToReef(DriveSubsystem subsystem, ReefAlignment reefAlignment) {
    m_subsystem = subsystem;
    this.reefAlignment = reefAlignment;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(reefAlignment){
      case LEFT_BRANCH:
        nearestBranch = m_subsystem.getPose().nearest(Coordinates.leftBranchCoordinates).plus(robotOffset);
        break;
      case RIGHT_BRANCH:
        nearestBranch = m_subsystem.getPose().nearest(Coordinates.rightBranchCoordinates).plus(robotOffset);
        break;
      case CENTER:
        nearestBranch = m_subsystem.getPose().nearest(Coordinates.reefCenterCoordinates).plus(robotOffset);
        break;
    }
    startWaypointPose = new Pose2d(m_subsystem.getPose().getTranslation(), nearestBranch.getTranslation().minus(m_subsystem.getPose().getTranslation()).getAngle());
    endWaypointPose = new Pose2d(nearestBranch.getTranslation(), nearestBranch.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        startWaypointPose,
        endWaypointPose
    );
    if(endWaypointPose.getTranslation().getDistance(startWaypointPose.getTranslation()) > 0.015){
      PathPlannerPath path = new PathPlannerPath(waypoints, PathPlannerConstants.pathConstraints, null, new GoalEndState(0, nearestBranch.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
      path.preventFlipping = true;
      pathfindCommand = m_subsystem.CommandBuilder.followPath(path);
      pathfindCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
