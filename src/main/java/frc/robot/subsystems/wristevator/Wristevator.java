package frc.robot.subsystems.wristevator;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SuperstructureConstants.WristevatorPreset;
import frc.robot.Constants.WristevatorConstants.Control;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wristevator.control.WristevatorController;
import frc.robot.subsystems.wristevator.control.WristevatorPIDController;

public class Wristevator {

    public static record WristevatorState(
        double elevatorHeightMeters,
        Rotation2d wristAngle,
        double elevatorVelMetPerSec,
        double wristVelRadPerSec
    ) {}

    public final ElevatorSubsystem m_elevator;
    public final WristSubsystem m_wrist;
    private final WristevatorController m_controller;

    public Wristevator(ElevatorSubsystem elevator, WristSubsystem wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_controller = new WristevatorPIDController(
            Control.wrist_kP,
            Control.wrist_kI,
            Control.wrist_kD,
            Control.elevator_kP,
            Control.elevator_kI,
            Control.elevator_kD,
            m_wrist::getAngle,
            m_elevator::getPositionMeters
        );
    }

    public void setKgConstants(double elevator_kG, double wrist_kG) {
        m_elevator.setKg(elevator_kG);
        m_wrist.setKg(wrist_kG);
    }

    public void applyState(WristevatorState state) {
        var speeds = m_controller.calculateSpeedsFromDesiredState(state);
        m_elevator.setVelocity(speeds.elevatorVelMetPerSec());
        m_wrist.setVelocity(speeds.wristVelRadPerSec());
    }

    public WristevatorState getCurrentState() {
        return new WristevatorState(
            m_elevator.getPositionMeters(), 
            m_wrist.getAngle(), 
            m_elevator.getVelocityMetersPerSec(), 
            m_wrist.getVelocityRadsPerSec()
        );
    }

    /** Allows the user to define a deploy safe height, so that the wrist can begin deploying as quickly as possible */
    public Command applyPresetDynamic(WristevatorPreset state, Supplier<Rotation2d> safeAngle, BooleanSupplier goingUp, double wristDeploySafeHeightMeters) {
        return Commands.either(
            Commands.parallel(
                m_elevator.applyPosition(state.elevatorHeightMeters),
                Commands.waitUntil(() -> m_elevator.getPositionMeters() >= wristDeploySafeHeightMeters)
                    .andThen(m_wrist.applyAngle(state.wristAngle))
            ).beforeStarting(m_wrist.applyAngle(safeAngle.get())
                .onlyIf(() -> m_elevator.getPositionMeters() < wristDeploySafeHeightMeters)
            ),
            Commands.sequence(
                Commands.parallel(
                    m_elevator.applyPosition(wristDeploySafeHeightMeters),
                    m_wrist.applyAngle(safeAngle.get())
                ).onlyIf(() -> m_elevator.getPositionMeters() > wristDeploySafeHeightMeters),
                Commands.parallel(
                    m_elevator.applyPosition(state.elevatorHeightMeters),
                    m_wrist.applyAngle(state.wristAngle)
                )
            ),
            goingUp
        );
    }
}