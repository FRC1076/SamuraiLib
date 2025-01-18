package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;


public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double leftMotorAppliedVoltage = 0;
        public double leftMotorCurrent = 0;

        public double rightMotorAppliedVoltage = 0;
        public double rightMotorCurrent = 0;
    }

    public default void updateInputs(GrabberIOInputs inputs) {}

    public default void runVolts(double volts) {}


    public default void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {}
}