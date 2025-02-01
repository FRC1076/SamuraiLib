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

    public abstract void updateInputs(ElevatorIOInputs inputs) ;

    public abstract void setPosition(double positionMeters) ;

    public abstract void setVoltage(double volts);

    public abstract void setVelocity(double velocityMetersPerSecond) ;

    public default void setFFkG(double kG) {}

    public default void simulationPeriodic() {}
}