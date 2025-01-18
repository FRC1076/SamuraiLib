package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {

        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;

        public double elevatorHeightMeters = 0;
        public double velocityMetersPerSecond = 0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double positionMeters) {}

    public default void setPosition(Distance positionMeters) {
        setPosition(positionMeters.in(Meters));
    }
    
    public default void setVelocity(double velocityMetersPerSecond) {}

    public default void setVelocity(LinearVelocity velocity) {
        setVelocity(velocity.in(MetersPerSecond));
    }

    public default void setVoltage(double volts) {}

    public default void setVoltage(Voltage voltage) {
        setVoltage(voltage.in(Volts));
    }
}