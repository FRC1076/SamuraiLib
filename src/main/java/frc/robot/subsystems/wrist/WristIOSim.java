package frc.robot.subsystems.wrist;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristSimConstants;

public class WristIOSim implements WristIO {
    private final DCMotor m_wristGearbox;

    private SparkMax m_leadMotor;
    private SparkMax m_followMotor;

    private final SparkMaxSim m_leadMotorSim;
    private final SparkMaxSim m_followMotorSim;

    private SparkMaxConfig m_leadMotorConfig;
    private SparkMaxConfig m_followMotorConfig;

    private final SparkRelativeEncoderSim m_encoderSim;

    private ArmFeedforward m_FFController = new ArmFeedforward(
        WristSimConstants.Control.kS,
        WristSimConstants.Control.kG,
        WristSimConstants.Control.kV,
        WristSimConstants.Control.kA
    );

    private final SingleJointedArmSim m_wristSim;

    private final SparkClosedLoopController m_closedLoopController;

    public WristIOSim() {
        m_wristGearbox = DCMotor.getNEO(2);

        m_leadMotor = new SparkMax(WristConstants.kLeadMotorPort, MotorType.kBrushless);
        m_followMotor = new SparkMax(WristConstants.kFollowMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        // create motor configurations
        m_leadMotorConfig
            .inverted(WristConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake);
            
        m_leadMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                WristConstants.Control.kP,
                WristConstants.Control.kI,
                WristConstants.Control.kD
            );

        m_leadMotorConfig.encoder
            .positionConversionFactor(WristConstants.kPositionConversionFactor)
            .velocityConversionFactor(WristConstants.kVelocityConversionFactor);
            

        m_followMotorConfig
            .follow(m_leadMotor)
            .inverted(WristConstants.kFollowMotorInverted)
            .idleMode(IdleMode.kBrake);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, null, null);
        m_followMotor.configure(m_followMotorConfig, null, null);

        m_wristSim = new SingleJointedArmSim(
            m_wristGearbox,
            WristSimConstants.kWristGearingReductions,
            SingleJointedArmSim.estimateMOI(WristSimConstants.kWristLength, WristSimConstants.kWristMass),
            WristSimConstants.kWristLength,
            WristSimConstants.kMinAngleRads,
            WristSimConstants.kMaxAngleRads,
            true,
            0,
            //WristSimConstants.kWristEncoderDistPerPulse,
            0.0,
            0.0
        );

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_wristGearbox);
        m_followMotorSim = new SparkMaxSim(m_followMotor, m_wristGearbox);
        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();

        //m_PIDController = new PIDController(WristSimConstants.Control.kP, WristSimConstants.Control.kI, WristSimConstants.Control.kD);
        m_closedLoopController = m_leadMotor.getClosedLoopController();
    }

    @Override
    public void simulationPeriodic() {
        m_wristSim.setInput(m_leadMotorSim.getAppliedOutput() * 12);
        m_wristSim.update(0.020);
        
        m_leadMotorSim.iterate(m_wristSim.getVelocityRadPerSec(),12,0.02);
        m_followMotorSim.iterate(m_wristSim.getVelocityRadPerSec(),12,0.02);
        //m_encoderSim.setPosition(m_wristSim.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        m_leadMotorSim.setAppliedOutput(voltage/12);
    }

    @Override
    public void setVelocity(double velocityRadiansPerSecond) {
        m_closedLoopController.setReference(
            velocityRadiansPerSecond,
            ControlType.kVelocity,
            m_leadMotorSim.getClosedLoopSlot(),
            m_FFController.calculateWithVelocities(
                m_wristSim.getAngleRads(),
                m_wristSim.getVelocityRadPerSec(),
                velocityRadiansPerSecond
            ),
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setPosition(double positionRadians) {
        m_closedLoopController.setReference(
            positionRadians,
            ControlType.kPosition,
            m_leadMotorSim.getClosedLoopSlot(),
            m_FFController.calculate(positionRadians, 0),
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * 12;
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.followCurrentAmps = m_followMotorSim.getMotorCurrent();
        inputs.angle = Rotation2d.fromRadians(m_encoderSim.getPosition());
    }
}
