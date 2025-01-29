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

    public abstract void updateInputs(GrabberIOInputs inputs);

    public abstract void runVolts(double volts);

    public abstract void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts);
}