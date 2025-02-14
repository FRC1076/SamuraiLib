package frc.robot.subsystems.leds;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDState;

// LED Driver that Controls the LEDs over a Parallel bus bitbanged from 3 GPIO pins
public class LEDParallel implements LEDDriver {
    /* Bitmasks */

    private static final byte mask0 = (byte) 0b00000001;
    private static final byte mask1 = (byte) 0b00000010;
    private static final byte mask2 = (byte) 0b00000100;
    private static final byte mask3 = (byte) 0b00001000;
    private static final byte mask4 = (byte) 0b00010000;
    private static final byte mask5 = (byte) 0b00100000;
    private static final byte mask6 = (byte) 0b01000000;
    private static final byte mask7 = (byte) 0b10000000;

    /* Declare an object for each pin */
    private final DigitalOutput m_pin0;
    private final DigitalOutput m_pin1;
    private final DigitalOutput m_pin2;

    /** Instantiate each digital pin, using integer constants for the channel */
    public LEDParallel() {
        m_pin0 = new DigitalOutput(LEDConstants.ParallelConfig.kDIOPort0);
        m_pin1 = new DigitalOutput(LEDConstants.ParallelConfig.kDIOPort1);
        m_pin2 = new DigitalOutput(LEDConstants.ParallelConfig.kDIOPort2);
    }

    @Override
    public void setState(LEDState state) {
        m_pin0.set((state.id & mask0) != 0);
        m_pin1.set((state.id & mask1) != 0);
        m_pin2.set((state.id & mask2) != 0);
    }
}
