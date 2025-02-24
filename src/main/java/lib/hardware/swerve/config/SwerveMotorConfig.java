package lib.hardware.swerve.config;

public class SwerveMotorConfig {
    public SlotConfig positionGains;
    public SlotConfig velocityGains;
    public SlotConfig currentGains;
    public double voltageComp;
    public double currentLimit;

    public SwerveMotorConfig withPositionGains(SlotConfig positionGains) {
        this.positionGains = positionGains;
        return this;
    }

    public SwerveMotorConfig withVelocityGains(SlotConfig velocityGains) {
        this.velocityGains = velocityGains;
        return this;
    }

    public SwerveMotorConfig withCurrentGains(SlotConfig currentGains) {
        this.currentGains = currentGains;
        return this;
    }

    public SwerveMotorConfig withVoltageComp(double voltageComp) {
        this.voltageComp = voltageComp;
        return this;
    }

    public SwerveMotorConfig withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }
}
