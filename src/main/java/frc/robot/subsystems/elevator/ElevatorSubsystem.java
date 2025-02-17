package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import lib.control.MutableElevatorFeedforward;
import lib.utils.MathHelpers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;

public class ElevatorSubsystem extends SubsystemBase {
    private static enum ControlMode {
        kVoltage,
        kVelocity,
        kPosition
    }
    private ControlMode mode = ControlMode.kVoltage;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController m_profiledPIDController = 
        new ProfiledPIDController(kP, kI, kD, new Constraints(1, 3));

    private final MutableElevatorFeedforward m_feedforwardController = new MutableElevatorFeedforward(kS, kG, kV, kA);

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
                break; //TODO: Velocity control has not yet been implemented
            case kPosition:
                io.setVoltage(
                    m_profiledPIDController.calculate(inputs.elevatorHeightMeters) + m_feedforwardController.getKg()
                );
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
        io.setPosition(MathHelpers.clamp(positionMeters, ElevatorConstants.kMinElevatorHeightMeters, ElevatorConstants.kMaxElevatorHeightMeters));
        //io.setPosition(positionMeters);
    }

    /** Set voltage of the elevator motors
     * @param volts Desired voltage of the elevator
     */
    public void setVoltage(double volts) {
        
        if (this.getPositionMeters() > ElevatorConstants.kMaxElevatorHeightMeters && volts > 0) {
            volts = m_feedforwardController.getKg();
        } else if (this.getPositionMeters() < ElevatorConstants.kMinElevatorHeightMeters && volts < 0) {
            volts = m_feedforwardController.getKg();
        }
    
        io.setVoltage(volts);
    }

    /** Set kG of the elevator's feedforward
     * @param kg New kG value in volts
     */
    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
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
            () -> {io.resetController();},
            () -> {setPosition(positionMeters);},
            (interrupted) -> {},
            () -> Math.abs(positionMeters - getPositionMeters()) < ElevatorConstants.elevatorPositionToleranceMeters,
            this
        );
    }
    
    /** Returns a command that sets the voltage of the elevator manually and adds kG
     * @param controlSupplier Supplier that returns the desired voltage of the elevator
     */
    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> {setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.maxOperatorControlVolts + io.getFFkG()); System.out.println(controlSupplier.getAsDouble());});
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.dynamic(direction);
    }

}
