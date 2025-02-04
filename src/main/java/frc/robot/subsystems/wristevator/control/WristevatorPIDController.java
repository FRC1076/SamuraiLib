package frc.robot.subsystems.wristevator.control;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;

public class WristevatorPIDController implements WristevatorController {
    
    private final PIDController wristController;
    private final PIDController elevatorController;
    
    private final Supplier<Rotation2d> wristPositionSupplier;
    private final DoubleSupplier elevatorPositionSupplier;
    
    public WristevatorPIDController(
        double wrist_kP,
        double wrist_kI,
        double wrist_kD,
        double elevator_kP,
        double elevator_kI,
        double elevator_kD,
        Supplier<Rotation2d> wristPositionSupplier,
        DoubleSupplier elevatorPositionSupplier
    ) {
        wristController = new PIDController(wrist_kP,wrist_kI,wrist_kD,0.02);
        elevatorController = new PIDController(elevator_kP,elevator_kI,elevator_kD,0.02);
        this.wristPositionSupplier = wristPositionSupplier;
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    @Override
    public WristevatorSpeeds calculateSpeedsFromDesiredState(WristevatorState state) {
        return new WristevatorSpeeds(
            elevatorController.calculate(elevatorPositionSupplier.getAsDouble(),state.elevatorHeightMeters()) + state.elevatorVelMetPerSec(),
            wristController.calculate(wristPositionSupplier.get().getRadians(),state.wristAngle().getRadians()) + state.wristVelRadPerSec()
        );
    }

}