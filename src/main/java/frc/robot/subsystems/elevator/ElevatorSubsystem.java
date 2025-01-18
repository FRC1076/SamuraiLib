package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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

    public void setPosition(Distance positionMeters) {
        io.setPosition(positionMeters);
    }

    public void setVelocity(double velocityMetersPerSecond){
        io.setVelocity(velocityMetersPerSecond);
    }

    public void setVelocity(LinearVelocity velocity){
        io.setVelocity(velocity);
    }

    public void setVoltage(double volts){
        io.setVoltage(volts);
    }

    /** Returns position of the elevator, as a Measure<Distance> */
    public Distance getPosition(){
        return Meters.of(inputs.elevatorHeightMeters);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator",inputs);
    }
}