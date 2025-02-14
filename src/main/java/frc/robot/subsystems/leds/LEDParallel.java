// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.leds;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDState;
import lib.hardware.parallel.ParallelOutput;

// LED Driver that Controls the LEDs over a Parallel bus bitbanged from 3 GPIO pins
public class LEDParallel implements LEDDriver {

    private final ParallelOutput bus;

    /** Instantiate each digital pin, using integer constants for the channel */
    public LEDParallel() {
        bus = new ParallelOutput(
            LEDConstants.ParallelConfig.kDIOPort0,
            LEDConstants.ParallelConfig.kDIOPort1,
            LEDConstants.ParallelConfig.kDIOPort2
        );
    }

    @Override
    public void setState(LEDState state) {
        bus.write(state.id);
    }
}
