package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged rearLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged rearRightInputs = new ModuleIOInputsAutoLogged();

    public DriveSubsystem(DriveIO io) {
        this.io = io;
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


}
