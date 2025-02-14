package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.LEDConstants.LEDState;

import static frc.robot.Constants.LEDConstants.SPIConfig.*;

// A driver for controlling LEDs over SPI
public class LEDSPI implements LEDDriver {
    private final SPI bus;
    public LEDSPI() {
        bus = new SPI(port);
        bus.setMode(mode);
        bus.setClockRate(clockSignalHz);
    }

    @Override
    public void setState(LEDState state) {
        bus.write(new byte[] {state.id},1);
    }
}
