package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDStates;

public class LEDSubsystem {
    private final LEDIO io;

    public LEDSubsystem(LEDIO io) {
        this.io = io;
    }

    public void setState(LEDStates state) {
        this.io.setState(state);
    }
}
