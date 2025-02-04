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