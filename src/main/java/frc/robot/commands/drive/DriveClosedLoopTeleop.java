package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
/**
 * Drives Teleop in closed loop mode from controller inputs
 */
public class DriveClosedLoopTeleop extends Command {
    private final DriveSubsystem m_subsystem;
    private final DoubleSupplier xTransSpeedSupplier; // field-oriented x translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier yTransSpeedSupplier; // field-oriented y translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier omegaSupplier; // rotational speed, scaled from -1.0 to 1.0
    private final BooleanSupplier lockedHeading;
    
    public DriveClosedLoopTeleop(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier lockedHeading, DriveSubsystem subsystem) {
        this.xTransSpeedSupplier = xSupplier;
        this.yTransSpeedSupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.m_subsystem = subsystem;
        this.lockedHeading = lockedHeading;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
            scaleSpeed(xTransSpeedSupplier.getAsDouble()) * 5,
            scaleSpeed(yTransSpeedSupplier.getAsDouble()) * 5,
            omegaSupplier.getAsDouble() * 2 * Math.PI
        );
        if(!lockedHeading.getAsBoolean()){
            m_subsystem.driveFO(
                speeds
            );
        }
        else{
            m_subsystem.driveFOHeadingLocked(
                speeds.vxMetersPerSecond, 
                speeds.vyMetersPerSecond, 
                new Translation2d(4.487, 4.010).minus(m_subsystem.getPose().getTranslation()).getAngle());
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        //m_subsystem.stop();
    }

    public double scaleSpeed(double speed){
        return speed / Math.max(Math.sqrt(Math.pow(xTransSpeedSupplier.getAsDouble(), 2) + Math.pow(yTransSpeedSupplier.getAsDouble(), 2)), 1);
    }

    public Command setState(){
        return Commands.runOnce(null, null);
    }
}