package frc.robot.subsystems.led;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDOnRIOConstants;
import frc.robot.Constants.LEDConstants.LEDStates;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDOnRIO implements LEDBase {
    AddressableLED m_leds;
    AddressableLEDBuffer m_buffer;

    public LEDOnRIO() {
        m_leds = new AddressableLED(LEDOnRIOConstants.kPWMPort);
        m_buffer = new AddressableLEDBuffer(LEDOnRIOConstants.kLength);

        // Setting the length is intensive, so ONLY update data after this
        m_leds.setLength(m_buffer.getLength());

        m_leds.setData(m_buffer);
        m_leds.start();
    }

    @Override
    public void setState(LEDStates state) {
        if(state == LEDStates.EMPTY) {
            // Solid purple
            LEDPattern.solid(Color.kPurple)
                .atBrightness(Percent.of(50))
                .applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.CORAL_INDEX) {
            // Flashing purple
            LEDPattern.solid(Color.kPurple)
                .atBrightness(Percent.of(100))
                .blink(Seconds.of(0.75))
                .applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.CORAL_GRABBER) {
            // Flashing white
            LEDPattern.solid(Color.kWhite)
                .atBrightness(Percent.of(100))
                .blink(Seconds.of(0.75))
                .applyTo(m_buffer);
            m_leds.setData(m_buffer);
        } else if (state == LEDStates.ALGAE) {
            // Flashing green
            LEDPattern.solid(Color.kGreen)
                .atBrightness(Percent.of(100))
                .blink(Seconds.of(0.75))
                .applyTo(m_buffer);
            m_leds.setData(m_buffer);
        }
    }
}