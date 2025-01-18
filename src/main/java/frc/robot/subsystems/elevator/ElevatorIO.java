package frc.robot.subsystems.elevator;
import org.littletonrobotics.junction.AutoLog;

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

    public default void setVoltage(double volts) {}
}