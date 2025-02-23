package lib.hardware.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A "plug-and-play" swerve drive subsystem, with built-in high frequency odometry, pathplanner, and AdvantageKit logging, as well as a
 * easily-extensible and highly polymorphic command system allowing for easy integration with virtually any codebase
 */
public class SamuraiSwerve extends SubsystemBase {
    
    private final SwerveDrive drive;
    
    public SamuraiSwerve(SwerveDrive drive) {
        this.drive = drive;
    }

}
