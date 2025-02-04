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
import frc.robot.subsystems.wristevator.control.WristevatorController.WristevatorSpeeds;
import frc.robot.commands.wristevator.FollowTrajectoryCommand;

import java.util.List;

public class Wristevator {

    public static record WristevatorState(
        double elevatorHeightMeters,
        Rotation2d wristAngle,
        double elevatorVelMetPerSec,
        double wristVelRadPerSec
    ) {}

    public final ElevatorSubsystem m_elevator;
    public final WristSubsystem m_wrist;
    public final WristevatorCommandFactory CommandBuilder;

    public Wristevator(ElevatorSubsystem elevator, WristSubsystem wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        CommandBuilder = new WristevatorCommandFactory(
            this,
            new WristevatorPIDController(
                Control.wrist_kP,
                Control.wrist_kI,
                Control.wrist_kD,
                Control.elevator_kP,
                Control.elevator_kI,
                Control.elevator_kD,
                Control.wristVelMOE,
                Control.wristPosMOE,
                Control.elvtrVelMOE,
                Control.elvtrPosMOE
            )
        );
    }

    public void setKgConstants(double elevator_kG, double wrist_kG) {
        m_elevator.setKg(elevator_kG);
        m_wrist.setKg(wrist_kG);
    }

    public void applySpeeds(WristevatorSpeeds speeds) {
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

    public static class WristevatorCommandFactory {
        private final Wristevator wristevator;
        private final WristevatorController controller;
        private WristevatorCommandFactory(Wristevator wristevator, WristevatorController controller) {
            this.wristevator = wristevator;
            this.controller = controller;
        }

        public Command applyPresetSequential(WristevatorPreset state, Supplier<Rotation2d> safeAngle) {
            return Commands.sequence(
                wristevator.m_wrist.applyAngle(safeAngle.get()),
                wristevator.m_elevator.applyPosition(state.elevatorHeightMeters),
                wristevator.m_wrist.applyAngle(state.wristAngle)
            );
        }

        /** Allows the user to define a deploy safe height, so that the wrist can begin deploying as quickly as possible */
        public Command applyPresetDynamic(WristevatorPreset state, Supplier<Rotation2d> safeAngle, BooleanSupplier goingUp, double wristDeploySafeHeightMeters) {
            return Commands.either(
                Commands.parallel(
                    wristevator.m_elevator.applyPosition(state.elevatorHeightMeters),
                    Commands.waitUntil(() -> wristevator.m_elevator.getPositionMeters() >= wristDeploySafeHeightMeters)
                        .andThen(wristevator.m_wrist.applyAngle(state.wristAngle))
                ).beforeStarting(wristevator.m_wrist.applyAngle(safeAngle.get())
                    .onlyIf(() -> wristevator.m_elevator.getPositionMeters() < wristDeploySafeHeightMeters)
                ),
                Commands.sequence(
                    Commands.parallel(
                        wristevator.m_elevator.applyPosition(wristDeploySafeHeightMeters),
                        wristevator.m_wrist.applyAngle(safeAngle.get())
                    ).onlyIf(() -> wristevator.m_elevator.getPositionMeters() > wristDeploySafeHeightMeters),
                    Commands.parallel(
                        wristevator.m_elevator.applyPosition(state.elevatorHeightMeters),
                        wristevator.m_wrist.applyAngle(state.wristAngle)
                    )
                ),
                goingUp
            );
        }

        public Command followTrajectory(List<WristevatorState> trajectory) {
            return new FollowTrajectoryCommand(
                wristevator,
                controller,
                trajectory
            );
        }

    }

    
}