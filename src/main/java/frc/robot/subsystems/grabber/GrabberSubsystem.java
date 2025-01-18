package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
    }

    public void runVolts(double volts) {
        this.io.runVolts(volts);
    }

    public void runVolts(Voltage voltage) {
        runVolts(voltage.in(Volts));
    }

    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {
        this.io.runVoltsDifferential(leftMotorVolts, rightMotorVolts);
    }

    public void runVoltsDifferential(Voltage leftMotorVoltage, Voltage rightMotorVoltage) {
        this.io.runVoltsDifferential(leftMotorVoltage.in(Volts), rightMotorVoltage.in(Volts));
    }

    public void stop() {
        runVolts(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }
    
}