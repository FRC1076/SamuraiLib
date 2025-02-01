package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class Leds extends SubsystemBase {
    private final SerialPort m_serialPort;

    public Leds() {
        m_serialPort = new SerialPort(LedConstants.kBaudRate, LedConstants.kLedPort);
    }
}
