// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.swerve.motors;

import lib.hardware.swerve.config.SwerveMotorConfig;
import lib.hardware.swerve.encoders.SwerveEncoder;

//TODO: Add relative encoder support
public interface SwerveMotor {
    public abstract void applyConfig(SwerveMotorConfig config);

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double position);

    public abstract void setVelocity(double velocity);

    public abstract void setCurrent(double current);

    public abstract SwerveEncoder getAbsoluteEncoder();
    
}
