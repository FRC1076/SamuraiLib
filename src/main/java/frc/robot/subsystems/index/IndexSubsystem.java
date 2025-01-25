package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
    private final IndexIO io;
    private final IndexIOInputsAutoLogged inputs = new IndexIOInputsAutoLogged();

    public IndexSubsystem(IndexIO io) {
        this.io = io;
    }

    public void runVolts(double volts) {
        this.io.runVolts(volts);
    }

    public void stop() {
        this.io.runVolts(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Index", inputs);
    }
}
