package lib.hardware.swerve.config;

import java.lang.module.ModuleDescriptor;
import java.util.Optional;
import java.util.OptionalInt;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.geometry.Translation2d;

public class SamuraiSwerveConfig {

    public static enum MotorType {
        kSparkMax,
        kSparkFlex,
        kTalonFX, // TODO: Implement
        kTalonFXS, // TODO: Implement
        kThriftyNova, // TODO: Implement
        kVenom, // TODO: Implement
        kTalonSRX, // TODO: Implement
    }

    public static enum GyroType {
        kADIS16448,
        kADIS16470,
        kADXRS450,
        kAnalog,
        kNavX,
        kPigeon2
    }

    public static enum AbsoluteEncoderType {
        kIntegrated, //Represents any absolute encoder directly connected to the steer motor controller
        kCANcoder,
        kCanandmag, // TODO: Implement
        kDutyCycleEncoder,
        kAnalogEncoder
    }

    public static class ModuleConfig {

        public MotorType driveMotorType;
        public MotorType steerMotorType;
        public AbsoluteEncoderType absoluteEncoderType;
        public int driveMotorID;
        public int steerMotorID;
        public int encoderID;
        public double absoluteEncoderOffset; //Rotations
        public SlotConfig driveGains;
        public SlotConfig steerGains;
        public double voltageComp;
        public double currentLimit;
        public double driveGearRatio;
        public double steerGearRatio;
        public double wheelRadius;
        public double maxModuleSpeed;

        public ModuleConfig withDriveMotorType(MotorType driveMotorType) {
            this.driveMotorType = driveMotorType;
            return this;
        }

        public ModuleConfig withSteerMotorType(MotorType steerMotorType) {
            this.steerMotorType = steerMotorType;
            return this;
        }

        public ModuleConfig withAbsoluteEncoderType(AbsoluteEncoderType absoluteEncoderType){
            this.absoluteEncoderType = absoluteEncoderType;
            return this;
        }

        public ModuleConfig withDriveMotorID(int driveMotorID){
            this.driveMotorID = driveMotorID;
            return this;
        }

        public ModuleConfig withSteerMotorID(int steerMotorID){
            this.steerMotorID = steerMotorID;
            return this;
        }

        public ModuleConfig withEncoderID(int encoderID){
            this.encoderID = encoderID;
            return this;
        }

        public ModuleConfig withDriveGearRatio(double driveGearRatio) {
            this.driveGearRatio = driveGearRatio;
            return this;
        }

        public ModuleConfig withSteerGearRatio(double steerGearRatio) {
            this.steerGearRatio = steerGearRatio;
            return this;
        }

        public ModuleConfig withWheelRadius(double wheelRadius) {
            this.wheelRadius = wheelRadius;
            return this;
        }

        public ModuleConfig withMaxModuleSpeed(double maxModuleSpeed) {
            this.maxModuleSpeed = maxModuleSpeed;
            return this;
        }

        public ModuleConfig withSteerGains(SlotConfig steerGains) {
            this.steerGains = steerGains;
            return this;
        }

        public ModuleConfig withDriveGains(SlotConfig driveGains) {
            this.driveGains = driveGains;
            return this;
        }

        public ModuleConfig withAbsoluteEncoderOffset(double offset) {
            this.absoluteEncoderOffset = offset;
            return this;
        }

        public ModuleConfig withVoltageComp(double voltageComp) {
            this.voltageComp = voltageComp;
            return this;
        }

        public ModuleConfig withCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

    }

    public static class ModuleConfigFactory {
        private MotorType driveMotorType;
        private MotorType steerMotorType;
        private AbsoluteEncoderType absoluteEncoderType;
        private SlotConfig driveGains;
        private SlotConfig steerGains;
        private double driveGearRatio;
        private double steerGearRatio;
        private double wheelRadius;
        private double maxModuleSpeed;
        private double voltageComp;
        private double currentLimit;

        public ModuleConfigFactory withDriveMotorType(MotorType driveMotorType) {
            this.driveMotorType = driveMotorType;
            return this;
        }

        public ModuleConfigFactory withSteerMotorType(MotorType steerMotorType) {
            this.steerMotorType = steerMotorType;
            return this;
        }

        public ModuleConfigFactory withAbsoluteEncoderType(AbsoluteEncoderType absoluteEncoderType){
            this.absoluteEncoderType = absoluteEncoderType;
            return this;
        }

        public ModuleConfigFactory withDriveGearRatio(double driveGearRatio) {
            this.driveGearRatio = driveGearRatio;
            return this;
        }

        public ModuleConfigFactory withSteerGearRatio(double steerGearRatio) {
            this.steerGearRatio = steerGearRatio;
            return this;
        }

        public ModuleConfigFactory withWheelRadius(double wheelRadius) {
            this.wheelRadius = wheelRadius;
            return this;
        }

        public ModuleConfigFactory withMaxModuleSpeed(double maxModuleSpeed) {
            this.maxModuleSpeed = maxModuleSpeed;
            return this;
        }

        public ModuleConfigFactory withSteerGains(SlotConfig steerGains) {
            this.steerGains = steerGains;
            return this;
        }

        public ModuleConfigFactory withDriveGains(SlotConfig driveGains) {
            this.driveGains = driveGains;
            return this;
        }

        public ModuleConfigFactory withVoltageComp(double voltageComp) {
            this.voltageComp = voltageComp;
            return this;
        }

        public ModuleConfigFactory withCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

        public ModuleConfig buildModuleConfig(int driveMotorID, int steerMotorID, int encoderID, double encoderOffset) {
            return new ModuleConfig()
                .withDriveMotorType(driveMotorType)
                .withSteerMotorType(steerMotorType)
                .withAbsoluteEncoderType(absoluteEncoderType)
                .withDriveGains(driveGains)
                .withSteerGains(steerGains)
                .withDriveGearRatio(driveGearRatio)
                .withSteerGearRatio(steerGearRatio)
                .withWheelRadius(wheelRadius)
                .withMaxModuleSpeed(maxModuleSpeed)
                .withCurrentLimit(currentLimit)
                .withVoltageComp(voltageComp)
                .withDriveMotorID(driveMotorID)
                .withSteerMotorID(steerMotorID)
                .withEncoderID(encoderID)
                .withAbsoluteEncoderOffset(encoderOffset);
        }
    }

    public GyroType gyroType;
    public int gyroID;
    public double maxModuleSpeed;
    public double maxTransSpeed;
    public double maxRotationSpeed;
    public Translation2d[] moduleTranslations;
    public ModuleConfig FLModuleConfig;
    public ModuleConfig FRModuleConfig;
    public ModuleConfig RLModuleConfig;
    public ModuleConfig RRModuleConfig;

    public SamuraiSwerveConfig withGyroType(GyroType gyroType) {
        this.gyroType = gyroType;
        return this;
    }

    public SamuraiSwerveConfig withGyroID(int gyroID) {
        this.gyroID = gyroID;
        return this;
    }

    public SamuraiSwerveConfig withMaxModuleSpeed(double maxModuleSpeed) {
        this.maxModuleSpeed = maxModuleSpeed;
        return this;
    }

    public SamuraiSwerveConfig withMaxTransSpeed(double maxTransSpeed){
        this.maxTransSpeed = maxTransSpeed;
        return this;
    }

    public SamuraiSwerveConfig withMaxRotSpeed(double maxRotSpeed){
        this.maxRotationSpeed = maxRotSpeed;
        return this;
    }

    public SamuraiSwerveConfig withModuleTranslations(Translation2d[] moduleTranslations) {
        this.moduleTranslations = moduleTranslations;
        return this;
    }

    public SamuraiSwerveConfig withFLModuleConfig(ModuleConfig FLModuleConfig) {
        this.FLModuleConfig = FLModuleConfig;
        return this;
    }

    public SamuraiSwerveConfig withFRModuleConfig(ModuleConfig FRModuleConfig) {
        this.FRModuleConfig = FRModuleConfig;
        return this;
    }

    public SamuraiSwerveConfig withRLModuleConfig(ModuleConfig RLModuleConfig) {
        this.RLModuleConfig = RLModuleConfig;
        return this;
    }

    public SamuraiSwerveConfig withRRModuleConfig(ModuleConfig RRModuleConfig) {
        this.RRModuleConfig = RRModuleConfig;
        return this;
    }
}
