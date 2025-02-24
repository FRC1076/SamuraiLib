package lib.hardware.swerve.encoders;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class FunctionalEncoder implements SwerveEncoder {
    private final DoubleConsumer offsetConsumer;
    private final DoubleSupplier encoderSignal;

    public FunctionalEncoder(DoubleConsumer offsetConsumer, DoubleSupplier encoderSignal) {
        this.offsetConsumer = offsetConsumer;
        this.encoderSignal = encoderSignal;
    }

    @Override
    public void setOffset(double offset) {
        offsetConsumer.accept(offset);
    }

    @Override
    public double getRotations() {
        return encoderSignal.getAsDouble();
    }
}
