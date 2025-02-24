// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.swerve.requests;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.hardware.swerve.SwerveOptimizer;
import lib.hardware.swerve.SwerveDriveBase.ControlParameters;
import lib.hardware.swerve.SwerveDriveBase.SwerveState;

public interface SamuraiSwerveRequest {

    public static enum SwerveRequestType {
        kDriveFieldOriented,
        kDriveRobotOriented,
        //kDriveFieldHeadingLock, //TODO: WIP
        //kDriveRobotHeadingLock, //TODO: WIP
        kLockWheels
        //kPointWheels, //TODO: WIP
        //kIdle //TODO: WIP
    }
    
    public abstract SwerveModuleState[] getModuleStates(SwerveOptimizer optimizer, SwerveState state); // Returns module states to be applied to the swerve drive

    public abstract double[] getXForceFeedforwardsNewtons();

    public abstract double[] getYForceFeedforwardsNewtons();

    public abstract double[] getArbParams(); // Returns a doublearray containing relevant swerve request data, in order to fully leverage CTRE's swerve library

    public abstract SwerveRequestType getRequestType();
}
