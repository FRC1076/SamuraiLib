// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.swerve.encoders;

public interface SwerveEncoder {
    public abstract void setOffset(double offset);
    public abstract double getRotations();
}
