// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wristevator.control;

import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;

public interface WristevatorController {
    public static record WristevatorSpeeds(
        double elevatorVelMetPerSec,
        double wristVelRadPerSec
    ) {}

    public abstract void setSetpoint(WristevatorState setpoint);

    public abstract WristevatorSpeeds calculate(WristevatorState measurement);

    public abstract boolean atSetpoint(WristevatorState measurement);
}