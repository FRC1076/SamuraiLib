package lib.hardware.swerve.requests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.hardware.swerve.SwerveOptimizer;
import lib.hardware.swerve.SwerveDriveBase.ControlParameters;
import lib.hardware.swerve.SwerveDriveBase.SwerveState;

public class LockWheels implements SamuraiSwerveRequest {

    private static final Rotation2d FLRotation = Rotation2d.fromDegrees(45);
    private static final Rotation2d FRRotation = Rotation2d.fromDegrees(-45);
    private static final Rotation2d RLRotation = Rotation2d.fromDegrees(135);
    private static final Rotation2d RRRotation = Rotation2d.fromDegrees(-135);
    
    @Override
    public SwerveModuleState[] getModuleStates(SwerveOptimizer optimizer, SwerveState state) {
        SwerveModuleState[] moduleStates = {
            new SwerveModuleState(0,FLRotation),
            new SwerveModuleState(0,FRRotation),
            new SwerveModuleState(0,RLRotation),
            new SwerveModuleState(0,RRRotation)
        };
        return moduleStates;
    }

    @Override
    public SwerveRequestType getRequestType() {
        return SwerveRequestType.kLockWheels;
    }

    @Override
    public double[] getArbParams() {
        return null;
    }

    @Override
    public double[] getXForceFeedforwardsNewtons() {
        return new double[] {0,0,0,0};
    }

    @Override
    public double[] getYForceFeedforwardsNewtons() {
        return new double[] {0,0,0,0};
    }
}
