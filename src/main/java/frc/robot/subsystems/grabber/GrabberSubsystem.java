package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

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

    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {
        this.io.runVoltsDifferential(leftMotorVolts, rightMotorVolts);
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