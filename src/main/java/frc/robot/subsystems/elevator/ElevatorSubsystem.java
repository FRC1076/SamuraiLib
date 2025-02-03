package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    /*To Do:
        Add a separate set voltage function that utilizes kG
    */

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine m_elevatorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("Elevator/SysIdElevatorState", state.toString())
        ) , 
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltage(voltage.in(Volts)),
            null,
            this
        )
    );

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.dynamic(direction);
    }
    
    public void setPosition(double positionMeters) {
        io.setPosition(positionMeters);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setKg(double kg) {
        this.io.setFFkG(kg);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }

    public Command applyPosition(double positionMeters) {
        return new FunctionalCommand(
            () -> {},
            () -> setPosition(positionMeters),
            (interrupted) -> {},
            () -> Math.abs(positionMeters - getPositionMeters()) < ElevatorConstants.elevatorPositionToleranceMeters,
            this
        );
    }
    
    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.maxOperatorControlVolts + io.getFFkG()));
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
