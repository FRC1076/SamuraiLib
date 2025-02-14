// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;
        public Rotation2d angle = new Rotation2d();
        public double velocityRadsPerSec = 0;
    }

    public abstract void updateInputs(WristIOInputs inputs);

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double velocityRadsPerSec);

    public abstract void setVoltageCharacterization(double volts);

    public abstract void setPosition(double position);

    public default void setFFkG(double kG) {}

    public default double getFFkG() {return WristConstants.Control.kG;}

    public default void simulationPeriodic() {}
}
