package frc.robot.subsystems.index;

import org.littletonrobotics.junction.AutoLog;

public interface IndexIO {
    @AutoLog
    public static class IndexIOInputs {
        public double leadMotorAppliedVoltage = 0;
        public double leadMotorCurrent = 0;

        public double followMotorAppliedVoltage = 0;
        public double followMotorCurrent = 0;
    }

    public default void updateInputs(IndexIOInputs inputs) {}

    public default void runVolts(double volts) {}
}
