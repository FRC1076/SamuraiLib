package lib.hardware.swerve.motors;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import lib.hardware.swerve.config.SwerveMotorConfig;

public class SwerveSparkMaxBrushless implements SwerveMotor {
    private final SparkMax motor;
    private final SparkClosedLoopController controller;

    public SwerveSparkMaxBrushless(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        controller = motor.getClosedLoopController();
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

    setVoltage() {
        
    }
}
