package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants.LEDConstants.LEDState;

import static frc.robot.Constants.LEDConstants.SerialConfig.*;

// Communicates with the arduino over a serial bus

public class LEDSerial implements LEDDriver {
    private final SerialPort bus;
    public LEDSerial() {
        bus = new SerialPort(
            baudRate,
            port,
            dataBits,
            parity,
            stopBits
        );
    }

    @Override
    public void setState(LEDState state) {
        bus.write(new byte[] {state.id},1);
    }
}
