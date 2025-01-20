package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Constants.DriveConstants.PathPlannerConstants;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged rearLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged rearRightInputs = new ModuleIOInputsAutoLogged();

    public DriveSubsystem(DriveIO io) {
        this.io = io;
        try{
            AutoBuilder.configure(
            () -> driveInputs.Pose,
            (Pose2d pose) -> resetPose(pose),
            () -> driveInputs.Speeds,
            (speeds) -> driveCO(speeds),
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
        
    }

    @Override
    public void periodic(){
        io.periodic(); //currently just for calling sim
        io.updateInputs(driveInputs);
        io.updateModuleInputs(frontLeftInputs,0);
        io.updateModuleInputs(frontRightInputs,1);
        io.updateModuleInputs(rearLeftInputs,2);
        io.updateModuleInputs(rearRightInputs,3);
        Logger.processInputs("Drive", driveInputs);
        Logger.processInputs("Drive/FrontLeft",frontLeftInputs);
        Logger.processInputs("Drive/FrontRight",frontRightInputs);
        Logger.processInputs("Drive/RearLeft",rearLeftInputs);
        Logger.processInputs("Drive/RearRight",rearRightInputs);
    }

    public void driveCO(ChassisSpeeds speeds){
        io.acceptRequest(new ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public void driveFO(ChassisSpeeds speeds){
        io.acceptRequest(new ApplyFieldSpeeds().withSpeeds(speeds));
    }

    public void driveFOHeadingLocked(double xMetersPerSecond, double yMetersPerSecond, Rotation2d targetDirection){
        FieldCentricFacingAngle request = new FieldCentricFacingAngle()
        .withVelocityX(xMetersPerSecond)
        .withVelocityY(yMetersPerSecond)
        .withTargetDirection(targetDirection);
        request.HeadingController.setPID(3.5, 0, 0);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        io.acceptRequest(
            request
        );
    }

    public void resetPose(Pose2d pose){
        io.resetPose(pose);
    }

    public Pose2d getPose(){
        return io.getPose();
    }

    public Command getPathfindToPoseCommand(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
            targetPose,
            PathPlannerConstants.pathConstraints,
            0.0
        );
    }

    public Command getFollowPathCommand(PathPlannerPath path){
        return AutoBuilder.followPath(path);
    }

}
