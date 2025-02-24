package lib.hardware.swerve.config;

public class SlotConfig {

    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;

    public SlotConfig withP(double kP) {
        this.kP = kP;
        return this;
    }

    public SlotConfig withI(double kI) {
        this.kI = kI;
        return this;
    }

    public SlotConfig withD(double kD) {
        this.kD = kD;
        return this;
    }

    public SlotConfig withS(double kS) {
        this.kS = kS;
        return this;
    }

    public SlotConfig withV(double kV) {
        this.kV = kV;
        return this;
    }

    public SlotConfig withA(double kA) {
        this.kA = kA;
        return this;
    }

}
