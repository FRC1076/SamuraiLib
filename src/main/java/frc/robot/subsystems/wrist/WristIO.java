package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVoltage(Voltage voltage) {
        setVoltage(voltage.in(Volts));
    }
}
