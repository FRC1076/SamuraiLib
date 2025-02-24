package lib.hardware.swerve;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.hardware.swerve.requests.SamuraiSwerveRequest;

public interface SwerveDriveBase {

    public static class SwerveState {
        
        public int successfulDaqs = 0;
        public int failedDaqs = 0;
        public Pose2d pose = new Pose2d();

        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {};
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {};
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[] {};
        
        public double timestamp = 0;
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public Rotation2d rawHeading = new Rotation2d();
        public double odometryPeriod = 0;

        public SwerveState clone() {
            var clone = new SwerveState();
            clone.successfulDaqs = successfulDaqs;
            clone.failedDaqs = failedDaqs;
            clone.pose = pose;
            clone.modulePositions = modulePositions.clone();
            clone.moduleStates = moduleStates.clone();
            clone.moduleTargets = moduleTargets.clone();
            clone.timestamp = timestamp;
            clone.speeds = speeds;
            clone.rawHeading = rawHeading;
            clone.odometryPeriod = odometryPeriod;
            return clone;
        }

    }

    public static record ControlParameters(
        double maxModuleSpeed,
        double maxTransSpeed,
        double maxRotSpeed
    ) {}

    public abstract void acceptRequest(SamuraiSwerveRequest request);

    public abstract void registerTelemetry(Consumer<SwerveState> telemetryConsumer);

    public abstract SwerveState getState();

    public abstract SwerveState getStateClone();

    public abstract void resetPose(Pose2d pose);

    public abstract void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs);

    public abstract void tareEverything();

    public abstract Optional<Pose2d> samplePoseAt(double timestamp);

    public abstract SwerveDriveKinematics getKinematics();

}
