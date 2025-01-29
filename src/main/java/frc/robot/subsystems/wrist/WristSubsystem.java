package frc.robot.subsystems.wrist;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

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


    public void stop() {
        setVoltage(0);
    }

    public double getAngleRadians() {
        return getAngle().getRadians();
    }

    public Command applyAngle(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {},
            () -> setPosition(angle), 
            (interrupted) -> {}, 
            () -> Math.abs(angle.minus(getAngle()).getRadians()) > WristConstants.wristAngleToleranceRadians,
            this
        );
    }

    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * WristConstants.maxOperatorControlVolts));
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
