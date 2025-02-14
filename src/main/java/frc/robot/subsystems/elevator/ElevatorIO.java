// Copyright (c) FRC1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;

import org.littletonrobotics.junction.AutoLog;

/**
 * A common interface that allows the subsystem code to interact with an elevator,
 * while abstracting away implementation, to allow for polymorphism (sim or hardware implementations)
 */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {

        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;

        public double elevatorHeightMeters = 0;
        public double velocityMetersPerSecond = 0;
    }

    /** Used for logging */
    public abstract void updateInputs(ElevatorIOInputs inputs) ;

    public abstract void setPosition(double positionMeters) ;

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double velocityMetersPerSecond);

    /** Sets the elevator controller's internal kG constant */
    public default void setFFkG(double kG) {}

    /** Retrieves the elevator controller's internal kG constant */
    public default double getFFkG() {return ElevatorConstants.Control.kG;}

    public default void simulationPeriodic() {}
}