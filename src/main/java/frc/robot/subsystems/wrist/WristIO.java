package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setPosition(double position) {}
}
