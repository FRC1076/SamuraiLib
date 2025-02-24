// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import lib.hardware.swerve.config.SlotConfig;
import lib.hardware.swerve.config.SwerveMotorConfig;
import lib.hardware.swerve.config.SamuraiSwerveConfig.ModuleConfig;
import lib.hardware.swerve.encoders.SwerveCANcoder;
import lib.hardware.swerve.encoders.SwerveEncoder;
import lib.hardware.swerve.motors.SwerveMotor;
import lib.hardware.swerve.motors.SwerveSparkMaxBrushless;

//TODO: WORK IN PROGRESS, DO NOT USE
public class SamuraiSwerveModule {
    private final SwerveMotor driveMotor;
    private final SwerveMotor steerMotor;
    private final SwerveEncoder encoder;

    public SamuraiSwerveModule(ModuleConfig config) {

        switch (config.driveMotorType) {
            case kSparkMax:
                driveMotor = new SwerveSparkMaxBrushless(config.driveMotorID);
                break;
            default:
                driveMotor = null;
        }

        switch (config.steerMotorType) {
            case kSparkMax:
                steerMotor = new SwerveSparkMaxBrushless(config.driveMotorID);
                break;
            default:
                steerMotor = null;
        }

        switch (config.absoluteEncoderType) {
            case kCANcoder:
                encoder = new SwerveCANcoder(config.encoderID);
                break;
            default:
                encoder = null;
        }

        var driveMotorConfig = new SwerveMotorConfig()
            .withVelocityGains(config.driveGains)
            .withPositionGains(new SlotConfig())
            .withCurrentGains(new SlotConfig())
            .withCurrentLimit(config.currentLimit)
            .withVoltageComp(config.voltageComp);

        var steerMotorConfig = new SwerveMotorConfig()
            .withPositionGains(config.steerGains)
            .withVelocityGains(new SlotConfig())
            .withCurrentGains(new SlotConfig())
            .withCurrentLimit(config.currentLimit)
            .withVoltageComp(config.voltageComp);
        
        steerMotor.applyConfig(steerMotorConfig);
        driveMotor.applyConfig(driveMotorConfig);

        encoder.setOffset(config.absoluteEncoderOffset);
    }

    public void setDesiredState(SwerveModuleState state) {

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }


}
