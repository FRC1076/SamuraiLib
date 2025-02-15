package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import lib.control.MutableElevatorFeedforward;
import lib.utils.MathHelpers;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

public class ElevatorSubsystem extends SubsystemBase {
    private static enum ControlMode {
        kVoltage,
        kVelocity,
        kPosition
    }
    private ControlMode mode = ControlMode.kVoltage;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final MutableElevatorFeedforward FeedforwardController = new MutableElevatorFeedforward(kS, kG, kV, kA);
    private final PIDController FeedbackController = new PIDController(kP,kI,kD);
    private final SysIdRoutine m_elevatorSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(2.5), // Reduce dynamic step voltage to 4 V to prevent brownout TODO: test if 4 V is enough
            null,        // Use default timeout (10 s)
            // Log state with AdvantageKit
            (state) -> Logger.recordOutput("Elevator/SysIDState", state.toString())
        ) , 
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltage(voltage.in(Volts)), // Voltage consumer for the SysID routine (Represents a function that SysID uses to pass voltages to the subsystem being characterized)
            null,
            this
        )
    );

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        //System.out.println("Elevator: " + this.getPositionMeters());
        io.updateInputs(inputs);
        Logger.processInputs("Elevator",inputs);
        switch (mode) {
            case kVoltage:
                break;
            case kVelocity:
                //TODO: Implement velocity control
                break;
            case kPosition:
                io.setVoltage(FeedbackController.calculate(inputs.elevatorHeightMeters) + FeedforwardController.getKg());
        }
    }

    @Override
    public void simulationPeriodic(){
        io.simulationPeriodic();
    }
    
    /** Set desired position of the elevator
     * @param positionMeters Desired position of the elevator in meters
     */
    /** TODO: VERY IMPORTANT: ADD SOFTWARE STOPS (where else?) */
    public void setPosition(double positionMeters) {
        mode = ControlMode.kPosition;
        FeedbackController.setSetpoint(positionMeters);
        //io.setPosition(positionMeters);
    }

    /** Set voltage of the elevator motors
     * @param volts Desired voltage of the elevator
     */
    public void setVoltage(double volts) {
        mode = ControlMode.kVoltage;
        if(this.getPositionMeters() > ElevatorConstants.kMaxElevatorHeightMeters && volts > 0) {
            volts = 0; //TODO: make this kG instead of 0?
        }
        else if(this.getPositionMeters() < ElevatorConstants.kMinElevatorHeightMeters && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts);
    }

    /** Set kG of the elevator's feedforward
     * @param kg New kG value in volts
     */
    public void setKg(double kg) {
        FeedforwardController.setKg(kg);
    }

    /** Returns position of the elevator, as a double */
    public double getPositionMeters(){
        return inputs.elevatorHeightMeters;
    }

    
    /* ######################################################################## */
    /* # Public Command Factories                                             # */
    /* ######################################################################## */
    
    /** Returns a command that sets the position of the elevator
     * @param positionMeters Desired position of the elevator in meters
     */
    public Command applyPosition(double positionMeters) {
        return new FunctionalCommand(
            () -> setPosition(positionMeters),
            () -> {},
            (interrupted) -> {},
            () -> Math.abs(positionMeters - getPositionMeters()) < ElevatorConstants.elevatorPositionToleranceMeters,
            this
        );
    }
    
    /** Returns a command that sets the voltage of the elevator manually and adds kG
     * @param controlSupplier Supplier that returns the desired voltage of the elevator
     */
    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.maxOperatorControlVolts + FeedforwardController.getKg()));
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.dynamic(direction);
    }

}
