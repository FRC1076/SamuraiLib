package frc.robot.subsystems.wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSimConstants;
import frc.robot.Constants.WristConstants.WristSimConstants;

public class WristIOSim implements WristIO {
    private final DCMotor m_wristGearbox;

    private SparkMax m_leadMotor;
    private SparkMax m_followMotor;

    private final SparkMaxSim m_leadMotorSim;
    private final SparkMaxSim m_followMotorSim;

    private SparkMaxConfig m_leadMotorConfig;
    private SparkMaxConfig m_followMotorConfig;

    private final SparkMaxAlternateEncoderSim m_encoderSim;

    private SparkClosedLoopController m_closedLoopController;

    private RelativeEncoder m_alternateEncoder;

    private ArmFeedforward FFController = new ArmFeedforward(
        WristConstants.Control.kS,
        WristConstants.Control.kG,
        WristConstants.Control.kV,
        WristConstants.Control.kA
    );

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d;
    private final MechanismRoot2d m_wristPivot;
    private final MechanismLigament2d m_wristTower;
    private final MechanismLigament2d m_wrist;

    private final SingleJointedArmSim m_wristSim;

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
            .follow(m_leadMotor)
            .inverted(WristConstants.kFollowMotorInverted)
            .idleMode(IdleMode.kBrake);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, null, null);
        m_followMotor.configure(m_followMotorConfig, null, null);

        m_closedLoopController = m_leadMotor.getClosedLoopController();

        m_alternateEncoder = m_leadMotor.getAlternateEncoder();

        m_wristSim = new SingleJointedArmSim(
            m_wristGearbox,
            WristSimConstants.kWristGearingReductions,
            SingleJointedArmSim.estimateMOI(WristSimConstants.kWristLength, WristSimConstants.kWristMass),
            WristSimConstants.kWristLength,
            WristSimConstants.kMinAngleRads,
            WristSimConstants.kMaxAngleRads,
            true,
            0,
            WristSimConstants.kWristEncoderDistPerPulse,
            0.0
        );

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_wristGearbox);
        m_followMotorSim = new SparkMaxSim(m_followMotor, m_wristGearbox);
        m_encoderSim = m_leadMotorSim.getAlternateEncoderSim();
    
        m_mech2d = new Mechanism2d(60, 60);
        m_wristPivot = m_mech2d.getRoot("WristPivot", 30, 30);
        m_wristTower = m_wristPivot.append(new MechanismLigament2d("WristTower", 30, -90));
        m_wrist = m_wristPivot.append(
            new MechanismLigament2d(
                "Wrist",
                30,
                Units.radiansToDegrees(m_wristSim.getAngleRads()))
        );

        SmartDashboard.putData("Wrist Sim", m_mech2d);
        m_wristTower.setColor(new Color8Bit(Color.kTeal)); // set color to teal because I feel like it
    }

    @Override
    public void simulationPeriodic() {
        m_wristSim.setInput(m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());
        System.out.println("Wrist Output: " + m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());

        m_wristSim.update(0.020);

        m_encoderSim.setPosition(m_wristSim.getAngleRads());

        m_wrist.setAngle(Units.radiansToDegrees(m_wristSim.getAngleRads()));
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotorSim.setAppliedOutput(volts/m_leadMotorSim.getBusVoltage());
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.followCurrentAmps = m_followMotorSim.getMotorCurrent();
    }
}
