package frc.robot.subsystems.leds;

import edu.wpi.first.hal.I2CJNI;
import frc.robot.Constants.LEDConstants.I2CConstants;
import frc.robot.Constants.LEDConstants.LEDState;

// A driver that controls the LEDs over a Serial I2C bus
public class LEDI2C implements LEDDriver {

    private final int port;
    private final byte devAddr;

    public LEDI2C() {
        port =  I2CConstants.port.value;
        devAddr = (byte) I2CConstants.devAddr;
    }

    @Override
    public void setState(LEDState state) {
        I2CJNI.i2CWriteB(port,devAddr,new byte[] {state.id},(byte) 1);
    }
}
