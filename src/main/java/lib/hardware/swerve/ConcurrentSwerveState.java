package lib.hardware.swerve;

import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.hardware.swerve.SwerveDriveBase.SwerveState;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

class ConcurrentSwerveState {

    private Lock stateLock = new ReentrantLock();
    
    private volatile int successfulDaqs;
    private volatile int failedDaqs;
    private volatile Pose2d pose;

    private volatile SwerveModulePosition[] modulePositions;
    private volatile SwerveModuleState[] moduleStates;
    private volatile SwerveModuleState[] moduleTargets;

    private volatile double timestamp;
    private volatile ChassisSpeeds speeds;
    private volatile Rotation2d rawHeading;
    private volatile double odometryPeriod;

    public SwerveState get() {
        var state = new SwerveState();
        
        synchronized (stateLock) {
            state.timestamp = timestamp;
            state.successfulDaqs = successfulDaqs;
            state.failedDaqs = failedDaqs;
            state.modulePositions = modulePositions.clone();
            state.moduleStates = moduleStates.clone();
            state.moduleTargets = moduleTargets.clone();
            state.pose = pose;
            state.speeds = speeds;
            state.rawHeading = rawHeading;
            state.odometryPeriod = odometryPeriod;
        }

        return state;
    }

    public void logSuccessfulDaq(
        double timestamp,
        SwerveModulePosition[] modulePositions,
        SwerveModuleState[] moduleStates,
        Pose2d pose,
        ChassisSpeeds speeds,
        Rotation2d rawHeading
    ) {
        synchronized (stateLock) {
            this.timestamp = timestamp;
            this.modulePositions = modulePositions;
            this.moduleStates = moduleStates;
            this.pose = pose;
            this.speeds = speeds;
            this.rawHeading = rawHeading;
            this.successfulDaqs++;
        }
    }

    public void logFailedDaq() {
        synchronized (stateLock) {
            this.failedDaqs++;
        }
    }

    public void resetDaqs() {
        synchronized (stateLock) {
            this.successfulDaqs = 0;
            this.failedDaqs = 0;
        }
    }

}