// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.leds;

import frc.robot.Constants.LEDConstants.LEDState;

// An interface for sending signals to an LED microcontroller
public interface LEDDriver {
    public abstract void setState(LEDState state);
}
