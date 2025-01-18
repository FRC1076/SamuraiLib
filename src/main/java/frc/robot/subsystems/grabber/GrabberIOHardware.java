package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.GrabberConstants;

public class GrabberIOHardware implements GrabberIO{
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;

    public GrabberIOHardware() {
        // motor port constant is currently unknown. Change when known.
        m_leftMotor = new SparkMax(GrabberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_rightMotor = new SparkMax(GrabberConstants.kRightMotorPort, MotorType.kBrushless);
    }

    @Override
    public void runVolts(double volts) {
        m_leftMotor.setVoltage(volts);
        m_rightMotor.setVoltage(volts);
    }

    @Override
    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {
        m_leftMotor.setVoltage(leftMotorVolts);
        m_rightMotor.setVoltage(rightMotorVolts);
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
        inputs.leftMotorCurrent = m_leftMotor.getOutputCurrent();
        
        inputs.rightMotorAppliedVoltage = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage();
        inputs.rightMotorCurrent = m_rightMotor.getOutputCurrent();
    }
}