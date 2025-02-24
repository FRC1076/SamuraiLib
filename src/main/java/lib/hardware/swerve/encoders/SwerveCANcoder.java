package lib.hardware.swerve.encoders;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;

public class SwerveCANcoder implements SwerveEncoder {
    private final CANcoder cancoder;
    private final StatusSignal<Angle> absoluteSignal;

    public SwerveCANcoder(int id) {
        cancoder = new CANcoder(id);
        absoluteSignal = cancoder.getAbsolutePosition(true);
    }

    @Override
    public double getRotations() {
        return absoluteSignal.getValueAsDouble();
    }

    @Override
    public void setOffset(double offset) {
        cancoder.getConfigurator().apply(
            new MagnetSensorConfigs().withMagnetOffset(offset)
        );
    }
}
