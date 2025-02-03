package frc.robot.subsystems.grabber;

/** 
 * A simple boilerplate class to enable full superstructure simulation
 */
public class GrabberIOSim implements GrabberIO {
    //TODO: Add Current and full motor simulation

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    public GrabberIOSim() {}

    @Override
    public void runVolts(double volts) {
        leftAppliedVolts = volts;
        rightAppliedVolts = volts;
    }

    @Override
    public void runVoltsDifferential(double leftMotorVoltage, double rightMotorVoltage) {
        leftAppliedVolts = leftMotorVoltage;
        rightAppliedVolts = rightMotorVoltage;
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = leftAppliedVolts;
        inputs.rightMotorAppliedVoltage = rightAppliedVolts;
    }
    
}
