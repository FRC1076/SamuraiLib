// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package lib.hardware.swerve;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleBase {
    public abstract SwerveModuleState getState();
    public abstract SwerveModulePosition getPosition();
    public abstract void setDesiredState(SwerveModuleState state);
    public default BooleanSupplier getErrorSignal() {
        return () -> false;
    }
}
