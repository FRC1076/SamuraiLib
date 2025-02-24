package lib.hardware.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Optimizes ChassisSpeeds and module states for a Swerve drivetrain
 */
public class SwerveOptimizer {
    private final SwerveDriveKinematics kinematics;
    private final double maxModuleSpeed;
    private final double maxTransSpeed;
    private final double maxRotSpeed;
    private final Translation2d centerOfRotation;

    public SwerveOptimizer(SwerveDriveKinematics kinematics,double maxModuleSpeed,double maxTransSpeed,double maxRotSpeed,Translation2d centerOfRotation) {
        this.kinematics = kinematics;
        this.maxModuleSpeed = maxModuleSpeed;
        this.maxTransSpeed = maxTransSpeed;
        this.maxRotSpeed = maxRotSpeed;
        this.centerOfRotation = centerOfRotation;
    }

    /**
     * Calculates optimal module states for the given speeds. Note: this algorithm uses a Desaturate-Discretize-Desaturate process to minimize translational drift, as
     * simply desaturating wheel speeds after discretization introduces translational drift
     * @param speeds
     * @param currentStates
     * @return
     * An array containing optimized module states
     */
    public SwerveModuleState[] calculateModuleStates(ChassisSpeeds speeds) {
        var tmpStates = kinematics.toSwerveModuleStates(speeds,centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            tmpStates,
            speeds,
            maxModuleSpeed,
            maxTransSpeed,
            maxRotSpeed
        );
        var discSpeeds = ChassisSpeeds.discretize(
            kinematics.toChassisSpeeds(tmpStates), 
            0.02
        );
        var discStates = kinematics.toSwerveModuleStates(discSpeeds,centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            discStates,
            discSpeeds,
            maxModuleSpeed,
            maxTransSpeed,
            maxRotSpeed
        );
        return discStates;
    }

    public SwerveModuleState[] calculateModuleStatesNoDesaturation(ChassisSpeeds speeds) {
        var discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        var discStates = kinematics.toSwerveModuleStates(discSpeeds,centerOfRotation);
        return discStates;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}
