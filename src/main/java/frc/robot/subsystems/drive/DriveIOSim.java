// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;

import org.littletonrobotics.junction.Logger;

public class DriveIOSim extends SwerveDrivetrain<TalonFX,TalonFX,CANcoder> implements DriveIO {

    private static class moduleSignalStruct {
        public StatusSignal<Voltage> turnAppliedVolts;
        public StatusSignal<Voltage> driveAppliedVolts;
        public StatusSignal<Current> turnStatorCurrent;
        public StatusSignal<Current> driveStatorCurrent;
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        Logger.recordOutput("Drive/VisionPoseEstimate",visionRobotPoseMeters);
        Logger.recordOutput("Drive/VisionEstimateStdDevs",visionMeasurementStdDevs);
    }

    private moduleSignalStruct[] moduleSignals = new moduleSignalStruct[4];
    private ConcurrentLinkedQueue<SwerveDriveState> odometryCache = new ConcurrentLinkedQueue<>();
    private SwerveDriveState[] odomDrain;
    private int oldDaqs; //Number of successul daqs from previous main loop cycle
    protected AtomicInteger Daqs = new AtomicInteger(0);
    
    public DriveIOSim(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?,?,?>... moduleConstants){
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            drivetrainConstants,
            odometryUpdateFrequency,
            moduleConstants
        );
        super.registerTelemetry(
            (state) -> {
                odometryCache.offer(state);
                Daqs.incrementAndGet();
            }
        );
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
            super.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
        }
        else{
            super.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));
        }
        for (int i = 0; i < 4; i++){
            moduleSignalStruct sigStruct = new moduleSignalStruct();
            SwerveModule<TalonFX,TalonFX,CANcoder> module = getModule(i);
            sigStruct.driveAppliedVolts = module.getDriveMotor().getMotorVoltage(true);
            sigStruct.driveStatorCurrent = module.getDriveMotor().getStatorCurrent(true);
            sigStruct.turnAppliedVolts = module.getSteerMotor().getMotorVoltage(true);
            sigStruct.turnStatorCurrent = module.getSteerMotor().getStatorCurrent(true);
            moduleSignals[i] = sigStruct;
        }
    }

    public DriveIOSim(CommandSwerveDrivetrain constants){
        this (
            constants.DrivetrainConstants(), 
            250.0,
            constants.FrontLeft(),
            constants.FrontRight(),
            constants.RearLeft(),
            constants.RearRight()
        );
    }

    private int drainCache(){
        oldDaqs = Daqs.getAndSet(0);
        odomDrain = new SwerveDriveState[oldDaqs];
        for (int i = 0; i < oldDaqs; i++){
            odomDrain[i] = odometryCache.poll();
        }
        return oldDaqs;
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        int drainSize = drainCache();
        inputs.fromSwerveDriveState(getState());
        inputs.odometryTimestamps = new double[drainSize];
        inputs.odometryHeadings = new Rotation2d[drainSize];
        inputs.odometryPoses = new Pose2d[drainSize];
        inputs.odometrySpeeds = new ChassisSpeeds[drainSize];
        inputs.operatorForwardDirection = getOperatorForwardDirection();
        for (int i = 0; i < drainSize; i++) {
            inputs.odometryTimestamps[i] = odomDrain[i].Timestamp;
            inputs.odometryHeadings[i] = odomDrain[i].RawHeading;
            inputs.odometryPoses[i] = odomDrain[i].Pose;
            inputs.odometrySpeeds[i] = odomDrain[i].Speeds;
        }

    }

    @Override
    public void updateModuleInputs(ModuleIOInputs inputs, int moduleIndex) {
        SwerveModule<TalonFX,TalonFX,CANcoder> module = getModule(moduleIndex);
        moduleSignalStruct sigStruct = moduleSignals[moduleIndex];
        SwerveModulePosition position = module.getPosition(true);
        SwerveModuleState state = module.getCurrentState();

        inputs.drivePositionMeters = position.distanceMeters;
        inputs.driveVelocityMetersPerSec = state.speedMetersPerSecond;
        inputs.driveAppliedVolts = sigStruct.driveAppliedVolts.getValueAsDouble();
        inputs.driveStatorCurrentAmps = sigStruct.driveStatorCurrent.getValueAsDouble();

        inputs.turnPosition = state.angle;
        inputs.turnAppliedVolts = sigStruct.turnAppliedVolts.getValueAsDouble();
        inputs.turnStatorCurrentAmps = sigStruct.turnStatorCurrent.getValueAsDouble();
    }

    @Override
    public void acceptRequest(SwerveRequest request){
        super.setControl(request);
    }

    @Override
    public void resetPose(Pose2d pose){
        super.resetPose(pose);
    }

    @Override
    public void resetHeading(){
        super.seedFieldCentric();
    }

    @Override
    public Pose2d getPose(){
        return super.getState().Pose;
    }

    @Override
    public void setAllianceRotation(Rotation2d allianceRotation){
        setOperatorPerspectiveForward(allianceRotation);
    }

    @Override
    public void periodic(){
        updateSimState(0.02, 12);
    }

      /** Set the current limit for each drive motor. 
     * Stator current is the amount of current that is sent to the motor.
     * <p>
     * <b>WARNING:</b> This method is resource intensive. Do not call it every loop.
     * 
     * @param currentLimit The amount of current sent to each motor.
     */
    @Override
    public void setDriveStatorCurrentLimit(double currentLimit) {
        
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(currentLimit);
            
        for (var module : getModules()) {
            module.getDriveMotor().getConfigurator().apply(currentConfigs);
        }
    }
}
