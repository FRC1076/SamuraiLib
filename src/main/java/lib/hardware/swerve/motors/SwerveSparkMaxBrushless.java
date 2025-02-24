package lib.hardware.swerve.motors;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import lib.hardware.swerve.config.SwerveMotorConfig;
import lib.hardware.swerve.encoders.FunctionalEncoder;
import lib.hardware.swerve.encoders.SwerveEncoder;

public class SwerveSparkMaxBrushless implements SwerveMotor {
    private final SparkMax motor;
    private final SparkClosedLoopController controller;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private DoubleSupplier encoderSignal;

    public SwerveSparkMaxBrushless(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        controller = motor.getClosedLoopController();
        absoluteEncoder = motor.getAbsoluteEncoder();
    }

    //TODO: Add feedforward, make better configurator
    @Override
    public void applyConfig(SwerveMotorConfig config) {
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig
            .smartCurrentLimit((int) config.currentLimit)
            .voltageCompensation(config.voltageComp)
        .closedLoop
            .p(config.positionGains.kP,ClosedLoopSlot.kSlot0)
            .i(config.positionGains.kI,ClosedLoopSlot.kSlot0)
            .d(config.positionGains.kD,ClosedLoopSlot.kSlot0)
            .p(config.velocityGains.kP,ClosedLoopSlot.kSlot1)
            .i(config.velocityGains.kI,ClosedLoopSlot.kSlot1)
            .d(config.velocityGains.kD,ClosedLoopSlot.kSlot1)
            .p(config.currentGains.kP,ClosedLoopSlot.kSlot2)
            .i(config.currentGains.kI,ClosedLoopSlot.kSlot2)
            .d(config.currentGains.kD,ClosedLoopSlot.kSlot2);
        motor.configure(
            sparkConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double position) {
        controller.setReference(position,ControlType.kPosition,ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setVelocity(double velocity) {
        controller.setReference(velocity,ControlType.kPosition,ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setCurrent(double current) {
        controller.setReference(current,ControlType.kPosition,ClosedLoopSlot.kSlot0);
    }

    //TODO: Write a non-scuffed implementation for this
    @Override
    public SwerveEncoder getAbsoluteEncoder() {
        return new FunctionalEncoder(
            (offset) -> encoderSignal = () -> absoluteEncoder.getPosition() + offset,
            encoderSignal
        );
    }
}
