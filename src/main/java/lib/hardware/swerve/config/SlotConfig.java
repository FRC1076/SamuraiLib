// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.swerve.config;

public class SlotConfig {

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;

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
