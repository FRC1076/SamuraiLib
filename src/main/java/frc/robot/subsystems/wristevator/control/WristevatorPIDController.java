package frc.robot.subsystems.wristevator.control;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.wristevator.Wristevator.WristevatorState;

public class WristevatorPIDController implements WristevatorController {
    
    private final PIDController wristController;
    private final PIDController elevatorController;
    
    private final double wristVelMOE;
    private final double wristPosMOE;
    private final double elvtrVelMOE;
    private final double elvtrPosMOE;
    
    private WristevatorState setpoint;
    public WristevatorPIDController(
        double wrist_kP,
        double wrist_kI,
        double wrist_kD,
        double elevator_kP,
        double elevator_kI,
        double elevator_kD,
        double wristVelMOE,
        double wristPosMOE,
        double elvtrVelMOE,
        double elvtrPosMOE
    ) {
        wristController = new PIDController(wrist_kP,wrist_kI,wrist_kD,0.02);
        elevatorController = new PIDController(elevator_kP,elevator_kI,elevator_kD,0.02);
        this.wristVelMOE = wristVelMOE;
        this.wristPosMOE = wristPosMOE;
        this.elvtrVelMOE = elvtrVelMOE;
        this.elvtrPosMOE = elvtrPosMOE;
    }
    @Override
    public void setSetpoint(WristevatorState setpoint) {
        this.setpoint = setpoint;
    }
    @Override
    public WristevatorSpeeds calculate(WristevatorState measurement) {
        return new WristevatorSpeeds(
            elevatorController.calculate(setpoint.elevatorHeightMeters(),measurement.elevatorHeightMeters()) + setpoint.elevatorVelMetPerSec(),
            wristController.calculate(setpoint.wristAngle().getRadians(),measurement.wristAngle().getRadians()) + setpoint.wristVelRadPerSec()
        );
    }

    @Override
    public boolean atSetpoint(WristevatorState measurement) {
        return (Math.abs(setpoint.elevatorHeightMeters() - measurement.elevatorHeightMeters()) < elvtrPosMOE)
                    && (Math.abs(setpoint.elevatorVelMetPerSec() - measurement.elevatorVelMetPerSec()) < elvtrVelMOE)
                    && (Math.abs(setpoint.wristAngle().getRadians() - measurement.wristAngle().getRadians()) < wristPosMOE)
                    && (Math.abs(setpoint.wristVelRadPerSec() - measurement.wristVelRadPerSec()) < wristPosMOE);

    }

}