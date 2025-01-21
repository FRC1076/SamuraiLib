package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Constants.DriveConstants.DriverControlConstants;
import frc.robot.Constants.DriveConstants.HeadingMode;
import frc.robot.Constants.DriveConstants.ClutchMode;
import frc.robot.Constants.Coordinates;
/**
 * Drives Teleop in closed loop mode from controller inputs
 */
public class DriveClosedLoopTeleop extends Command {
    private final DriveSubsystem m_subsystem;
    private final DoubleSupplier xTransSpeedSupplier; // field-oriented x translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier yTransSpeedSupplier; // field-oriented y translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier omegaSupplier; // rotational speed, scaled from -1.0 to 1.0
    private HeadingMode headingMode = HeadingMode.NORMAL;
    private ClutchMode clutchMode = ClutchMode.NORMAL;
    
    public DriveClosedLoopTeleop(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, DriveSubsystem subsystem) {
        this.xTransSpeedSupplier = xSupplier;
        this.yTransSpeedSupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.m_subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
            scaleSpeed(xTransSpeedSupplier.getAsDouble()) * 5,
            scaleSpeed(yTransSpeedSupplier.getAsDouble()) * 5,
            omegaSupplier.getAsDouble() * 2 * Math.PI
        );
        switch(clutchMode){
            case NORMAL:
                break;
            case SINGLE_CLUTCH:
                speeds.vxMetersPerSecond *= DriverControlConstants.singleClutchTranslationFactor;
                speeds.vyMetersPerSecond *= DriverControlConstants.singleClutchTranslationFactor;
                speeds.omegaRadiansPerSecond *= DriverControlConstants.singleClutchRotationFactor;
                break;
            case DOUBLE_CLUTCH:
                speeds.vxMetersPerSecond *= DriverControlConstants.doubleClutchTranslationFactor;
                speeds.vyMetersPerSecond *= DriverControlConstants.doubleClutchTranslationFactor;
                speeds.omegaRadiansPerSecond *= DriverControlConstants.doubleClutchRotationFactor;
                break;
        }
        switch(headingMode){
            case NORMAL:
                m_subsystem.driveFO(speeds);
                break;
            case POINT_TO_REEF:
                Translation2d reefLocation;
                if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
                    reefLocation = Coordinates.redReefCenter;
                }
                else{
                    reefLocation = Coordinates.blueReefCenter;
                }
                Rotation2d angleToReef = reefLocation.minus(m_subsystem.getPose().getTranslation()).getAngle();
                if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
                    angleToReef = angleToReef.rotateBy(Rotation2d.fromDegrees(180));
                }
                m_subsystem.driveFOHeadingLocked(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    angleToReef
                );
                break;
            case LEFT_CORAL_STATION:
                m_subsystem.driveFOHeadingLocked(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    Coordinates.leftCoralStationAngle
                );
                break;
            case RIGHT_CORAL_STATION:
                m_subsystem.driveFOHeadingLocked(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    Coordinates.rightCoralStationAngle
                );
                break;
            case STRAIGHT:
                m_subsystem.driveFOHeadingLocked(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    Rotation2d.fromDegrees(0)
                );
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        //m_subsystem.stop();
    }

    public double scaleSpeed(double speed){
        return speed / Math.max(Math.sqrt(Math.pow(xTransSpeedSupplier.getAsDouble(), 2) + Math.pow(yTransSpeedSupplier.getAsDouble(), 2)), 1);
    }

    public Command setNormalClutchMode(){
        return Commands.runOnce(() -> {clutchMode = ClutchMode.NORMAL;});
    }

    public Command setSingleClutchMode(){
        return Commands.runOnce(() -> {clutchMode = ClutchMode.SINGLE_CLUTCH;});
    }

    public Command setDoubleClutchMode(){
        return Commands.runOnce(() -> {clutchMode = ClutchMode.DOUBLE_CLUTCH;});
    }

    public Command setNormalHeadingMode(){
        return Commands.runOnce(() -> {headingMode = HeadingMode.NORMAL;});
    }

    public Command setPointToReefHeadingMode(){
        return Commands.runOnce(() -> {headingMode = HeadingMode.POINT_TO_REEF;});
    }

    public Command setLeftCoralStationHeadingMode(){
        return Commands.runOnce(() -> {headingMode = HeadingMode.LEFT_CORAL_STATION;});
    }

    public Command setRightCoralStationHeadingMode(){
        return Commands.runOnce(() -> {headingMode = HeadingMode.RIGHT_CORAL_STATION;});
    }

    public Command setStraightHeadingMode(){
        return Commands.runOnce(() -> {headingMode = HeadingMode.STRAIGHT;});
    }
}