package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;
        public Rotation2d angle = new Rotation2d();
    }

    public abstract void updateInputs(WristIOInputs inputs);

    public abstract void setVoltage(double volts);

    public abstract void setPosition(double position);

    public default void setFFkG(double kG) {}

    public default double getFFkG() {return WristConstants.Control.kG;}

    public default void simulationPeriodic() {}
}
