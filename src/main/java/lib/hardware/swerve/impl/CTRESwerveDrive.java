package lib.hardware.swerve.impl;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.DeviceConstructor;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lib.hardware.swerve.SwerveDrive;
import lib.hardware.swerve.requests.SamuraiSwerveRequest;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CTRESwerveDrive<DriveMotorT extends CommonTalon, SteerMotorT extends CommonTalon, EncoderT extends ParentDevice> implements SwerveDrive {
    
    private final SwerveDrivetrain<DriveMotorT,SteerMotorT,EncoderT> drivetrain;
    private final SwerveState state = new SwerveState();

    //Converts from CTRE SwerveDriveState to SamuraiSwerve SwerveState
    private static final Function<SwerveDriveState,SwerveState> CTREToSamuraiState = (ctreState) -> {
        var state = new SwerveState();
        state.failedDaqs = ctreState.FailedDaqs;
        state.successfulDaqs = ctreState.SuccessfulDaqs;
        state.modulePositions = ctreState.ModulePositions.clone();
        state.moduleStates = ctreState.ModuleStates.clone();
        state.moduleTargets = ctreState.ModuleTargets.clone();
        state.pose = ctreState.Pose;
        state.rawHeading = ctreState.RawHeading;
        state.speeds = ctreState.Speeds;
        state.odometryPeriod = ctreState.OdometryPeriod;
        return state;
    };

    private static final Function<SwerveState,SwerveDriveState> SamuraiToCTREState = (samuraiState) -> {
        var state = new SwerveDriveState();
        state.FailedDaqs = samuraiState.failedDaqs;
        state.SuccessfulDaqs = samuraiState.successfulDaqs;
        state.ModulePositions = samuraiState.modulePositions.clone();
        state.ModuleStates = samuraiState.moduleStates.clone();
        state.ModuleTargets = samuraiState.moduleTargets.clone();
        state.Pose = samuraiState.pose;
        state.RawHeading = samuraiState.rawHeading;
        state.Speeds = samuraiState.speeds;
        state.OdometryPeriod = samuraiState.odometryPeriod;
        return state;
    };

    private void updateState() {
        var ctreState = drivetrain.getState();
        state.failedDaqs = ctreState.FailedDaqs;
        state.successfulDaqs = ctreState.SuccessfulDaqs;
        state.modulePositions = ctreState.ModulePositions.clone();
        state.moduleStates = ctreState.ModuleStates.clone();
        state.moduleTargets = ctreState.ModuleTargets.clone();
        state.pose = ctreState.Pose;
        state.rawHeading = ctreState.RawHeading;
        state.speeds = ctreState.Speeds;
        state.odometryPeriod = ctreState.OdometryPeriod;
    }

    public CTRESwerveDrive(
        DeviceConstructor<DriveMotorT> driveMotorConstructor,
        DeviceConstructor<SteerMotorT> steerMotorConstructor,
        DeviceConstructor<EncoderT> encoderConstructor,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        drivetrain = new SwerveDrivetrain<>(driveMotorConstructor, steerMotorConstructor, encoderConstructor, drivetrainConstants, modules);
    }

    @Override
    public void registerTelemetry(Consumer<SwerveState> telemetryConsumer){
        drivetrain.registerTelemetry((state) -> telemetryConsumer.accept(CTREToSamuraiState.apply(state)));
    }

    @Override
    public SwerveState getState() {
        updateState();
        return state;
    }

    @Override
    public SwerveState getStateClone() {
        updateState();
        return state.clone();
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    //TODO: Add support for all SamuraiSwerveRequest types
    public void applyRequest(SamuraiSwerveRequest request) {
        double[] arbParams = request.getArbParams();
        double[] xForceFeedforwards = request.getXForceFeedforwardsNewtons();
        double[] yForceFeedforwards = request.getYForceFeedforwardsNewtons();
        SwerveRequest CTRERequest;
        switch (request.getRequestType()) {
            case kDriveFieldOriented:
                CTRERequest = new SwerveRequest.ApplyFieldSpeeds()
                    .withSpeeds(new ChassisSpeeds(arbParams[0],arbParams[1],arbParams[2]))
                    .withCenterOfRotation(new Translation2d(arbParams[3],arbParams[4]))
                    .withDesaturateWheelSpeeds(arbParams[5] == 1.0)
                    .withWheelForceFeedforwardsX(xForceFeedforwards)
                    .withWheelForceFeedforwardsY(yForceFeedforwards);
                break;
            case kDriveRobotOriented:
                CTRERequest = new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(new ChassisSpeeds(arbParams[0],arbParams[1],arbParams[2]))
                    .withCenterOfRotation(new Translation2d(arbParams[3],arbParams[4]))
                    .withDesaturateWheelSpeeds(arbParams[5] == 1.0)
                    .withWheelForceFeedforwardsX(xForceFeedforwards)
                    .withWheelForceFeedforwardsY(yForceFeedforwards);
                break;
            case kLockWheels:
                CTRERequest = new SwerveRequest.SwerveDriveBrake();
                break;
            default:
                CTRERequest = new SwerveRequest.Idle();
        }
        drivetrain.setControl(CTRERequest);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    }

    
}
