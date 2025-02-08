package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSimConstants;
import static frc.robot.Constants.ElevatorConstants.kPositionConversionFactor;
import static frc.robot.Constants.ElevatorConstants.kVelocityConversionFactor;
import static frc.robot.Constants.ElevatorConstants.Electrical.kCurrentLimit;
import static frc.robot.Constants.ElevatorConstants.Electrical.kVoltageCompensation;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkRelativeEncoderSim;



public class ElevatorIOSimRev implements ElevatorIO {

    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

    private final SparkMax m_leadMotor;
    private final SparkMax m_followMotor;

    private final SparkMaxSim m_leadMotorSim;
    private final SparkMaxSim m_followMotorSim;

    private SparkMaxConfig m_leadMotorConfig;
    private SparkMaxConfig m_followMotorConfig;
    
    private final SparkRelativeEncoderSim m_encoderSim;

    private final ElevatorSim m_elevatorSim;

    private final SparkClosedLoopController m_feedbackController;

    private ElevatorFeedforward m_FFController = new ElevatorFeedforward(
        ElevatorConstants.Control.kS, 
        ElevatorConstants.Control.kG,
        ElevatorConstants.Control.kV, 
        ElevatorConstants.Control.kA
    );

    public ElevatorIOSimRev() {
        
        m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            ElevatorSimConstants.kElevatorGearing,
            ElevatorSimConstants.kCarriageMass,
            ElevatorSimConstants.kElevatorDrumRadius,
            ElevatorSimConstants.kMinElevatorHeightMeters,
            ElevatorSimConstants.kMaxElevatorHeightMeters,
            true,
            0,
            0,
            0
        );
    
        m_leadMotor = new SparkMax(ElevatorSimConstants.kSimMotorPort0, SparkMax.MotorType.kBrushless);
        m_followMotor = new SparkMax(ElevatorSimConstants.kSimMotorPort1, SparkMax.MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        m_leadMotor.setCANTimeout(250);
        m_followMotor.setCANTimeout(250);

        m_leadMotorConfig
            .inverted(ElevatorConstants.leadMotorInverted)
            .smartCurrentLimit((int) kCurrentLimit)
            .voltageCompensation(kVoltageCompensation);
        m_leadMotorConfig.closedLoop
            .pid(
                ElevatorConstants.Control.kPosP,
                ElevatorConstants.Control.kPosI,
                ElevatorConstants.Control.kPosD,
                ClosedLoopSlot.kSlot0
            )
            .pid(
                ElevatorConstants.Control.kVelP,
                ElevatorConstants.Control.kVelI,
                ElevatorConstants.Control.kVelD,
                ClosedLoopSlot.kSlot1
            )
            .p(1,ClosedLoopSlot.kSlot2);
        m_leadMotorConfig.encoder
            .positionConversionFactor(kPositionConversionFactor)
            .velocityConversionFactor(kVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        m_followMotorConfig
            .inverted(ElevatorConstants.followMotorInverted)
            .smartCurrentLimit((int) kCurrentLimit)
            .voltageCompensation(kVoltageCompensation)
            .follow(m_leadMotor);
        
        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_leadMotor.setCANTimeout(0);
        m_followMotor.setCANTimeout(0);

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_elevatorGearbox);
        m_followMotorSim = new SparkMaxSim(m_followMotor, m_elevatorGearbox);
        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();

        m_feedbackController = m_leadMotor.getClosedLoopController();
    }

    @Override
    public void setPosition(double positionMeters) {
        m_feedbackController.setReference(
            positionMeters,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            m_FFController.getKg(),
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setVelocity(double velocityMetersPerSecond) {
        m_feedbackController.setReference(
            velocityMetersPerSecond, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot1,
            m_FFController.calculate(velocityMetersPerSecond),
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setVoltage(double voltage) {
        m_feedbackController.setReference(
            voltage,
            ControlType.kVoltage,
            ClosedLoopSlot.kSlot2
        );
    }

    @Override 
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.followCurrentAmps = m_followMotorSim.getMotorCurrent();
        inputs.elevatorHeightMeters = m_encoderSim.getPosition();
        inputs.velocityMetersPerSecond = m_encoderSim.getVelocity();
    }

    @Override
    public void simulationPeriodic() {

        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_elevatorSim.setInput(m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_elevatorSim.update(0.020);

        m_leadMotorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(),12,0.02);
        m_followMotorSim.iterate(m_elevatorSim.getVelocityMetersPerSecond(), 12, 0.02);
    }
}