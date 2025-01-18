package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;


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

    public default void runVolts(Voltage voltage) {
        runVolts(voltage.in(Volts));
    }

    public default void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {}

    public default void runVoltsDifferential(Voltage leftMotorVoltage, Voltage rightMotorVoltage) {
        runVoltsDifferential(leftMotorVoltage.in(Volts), rightMotorVoltage.in(Volts));
    }
}