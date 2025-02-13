package frc.robot.subsystems.leds;

import frc.robot.Constants.LEDConstants.LEDState;

// An interface for sending signals to an LED microcontroller
public interface LEDDriver {
    public abstract void setState(LEDState state);
}
