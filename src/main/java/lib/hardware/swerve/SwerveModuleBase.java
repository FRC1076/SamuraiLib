package lib.hardware.swerve;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getPosition();
    public abstract void setDesiredState(SwerveModuleState state);
}
