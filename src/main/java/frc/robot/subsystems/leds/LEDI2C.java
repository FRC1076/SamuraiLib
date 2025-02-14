// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.leds;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants.LEDConstants.I2CConfig;
import frc.robot.Constants.LEDConstants.LEDState;

// A driver that controls the LEDs over an I2C bus
public class LEDI2C implements LEDDriver {

    private final I2C bus;

    public LEDI2C() {
        bus = new I2C(I2CConfig.port,I2CConfig.devAddr);
    }

    @Override
    public void setState(LEDState state) {
        bus.writeBulk(new byte[] {state.id});
    }
}
