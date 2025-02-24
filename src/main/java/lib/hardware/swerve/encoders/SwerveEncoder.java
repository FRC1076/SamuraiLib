package lib.hardware.swerve.encoders;

public interface SwerveEncoder {
    public abstract void setOffset(double offset);
    public abstract double getRotations();
}
