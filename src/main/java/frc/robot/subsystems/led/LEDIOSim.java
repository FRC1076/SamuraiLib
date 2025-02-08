package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants.LEDStates;

public class LEDIOSim implements LEDIO {
    @Override
    public void setState(LEDStates state) {
        // do nothing because there are no simulated LEDs
    }
}
