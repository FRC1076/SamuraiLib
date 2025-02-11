package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;

import lib.control.MutableArmFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class WristIOHardware implements WristIO {
    private final SparkMax m_leadMotor;
    private final SparkMax m_followMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkMaxConfig m_followMotorConfig;

    private final SparkClosedLoopController m_closedLoopController;

    private final RelativeEncoder m_alternateEncoder;

    private final MutableArmFeedforward FFController = new MutableArmFeedforward(
        WristConstants.Control.kS,
        WristConstants.Control.kG,
        WristConstants.Control.kV,
        WristConstants.Control.kA
    );

    public WristIOHardware() {
        m_leadMotor = new SparkMax(WristConstants.kLeadMotorPort, MotorType.kBrushless);
        m_followMotor = new SparkMax(WristConstants.kFollowMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();
        

        // create motor configurations
        m_leadMotorConfig
            .inverted(WristConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake);
            
        m_leadMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(
                WristConstants.Control.kP,
                WristConstants.Control.kI,
                WristConstants.Control.kD
            );

        m_leadMotorConfig.alternateEncoder
            .setSparkMaxDataPortConfig()
            .countsPerRevolution(WristConstants.kCountsPerRevolution)
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);
            

        m_followMotorConfig
            .follow(m_leadMotor, WristConstants.kFollowMotorInverted != WristConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_closedLoopController = m_leadMotor.getClosedLoopController();
        m_alternateEncoder = m_leadMotor.getAlternateEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts + FFController.calculate(m_alternateEncoder.getPosition(),0));
    }
    
    /** Set voltage of the wrist motors without the feedforward */
    @Override
    public void setVoltageCharacterization(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    /** TODO: VERY IMPORTANT: ADD SOFTWARE STOPS */
    @Override
    public void setPosition(double positionRadians){
        m_closedLoopController.setReference(
            positionRadians, // clamp(positionRadians, WristConstants.minPositionRadians, WristConstants.maxPositionRadians),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            FFController.calculate(positionRadians, 0), // these values might be wrong
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.followCurrentAmps = m_followMotor.getOutputCurrent();
        inputs.angle = Rotation2d.fromRadians(m_alternateEncoder.getPosition());
    }

    @Override
    public void setFFkG(double kG){
        FFController.setKg(kG);
    }

    @Override
    public double getFFkG() {
        return FFController.getKg();
    }

}
