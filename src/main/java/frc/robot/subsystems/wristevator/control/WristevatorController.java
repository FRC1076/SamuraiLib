package frc.robot.subsystems.wristevator.control;

import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;

public interface WristevatorController {
    public static record WristevatorSpeeds(
        double elevatorVelMetPerSec,
        double wristVelRadPerSec
    ) {}

    public abstract WristevatorSpeeds calculateSpeedsFromDesiredState(WristevatorState state);
}