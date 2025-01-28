package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }

    public void setPosition(double positionMeters) {
        io.setPosition(positionMeters);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }

    public Command applyPosition(double positionMeters) {
        return runOnce(() -> setPosition(positionMeters));
    }

    public Command applyVoltage(double volts) {
        return runOnce(() -> setVoltage(volts));
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator",inputs);
    }

    @Override
    public void simulationPeriodic(){
        io.simulationPeriodic();
    }
}
