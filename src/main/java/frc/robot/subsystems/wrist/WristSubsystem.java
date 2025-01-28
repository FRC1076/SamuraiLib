package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPosition(Rotation2d position) {
        io.setPosition(position.getRadians());
    }

    public Rotation2d getAngle(){
        return inputs.angle;
    }

    public Command applyVoltage(double volts) {
        return runOnce(() -> setVoltage(volts));
    }

    public Command applyPosition(Rotation2d position) {
        return runOnce(() -> setPosition(position));
    }

    public void stop() {
        setVoltage(0);
    }

    public double getAngleRadians() {
        return getAngle().getRadians();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
