// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;
import lib.control.DynamicArmFeedforward;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final ProfiledPIDController m_profiledPIDController;
    private final DynamicArmFeedforward m_feedforwardController;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final SysIdRoutine sysid;
    

    public WristSubsystem(WristIO io) {
        this.io = io;

        var controlConstants = io.getControlConstants();
        m_profiledPIDController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(), 
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new DynamicArmFeedforward(
            controlConstants.kS(),
            controlConstants.kG(),
            controlConstants.kV(),
            controlConstants.kA()
        );

        sysid = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(1), null,
                (state) -> Logger.recordOutput("Wrist/SysIDState", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );

    }
    
    /** Sets the voltage of the wrist motors, compensating for gravity*/
    public void setVoltage(double volts) {
        
        if (this.getAngleRadians() > WristConstants.kMaxWristAngleRadians && volts > 0) {
            volts = 0;
        } else if (this.getAngleRadians() < WristConstants.kMinWristAngleRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    /** Sets the desired rotation of the wrist */
    public void setAngle(Rotation2d position) {
        io.setVoltage(
            m_profiledPIDController.calculate(inputs.angleRadians,MathUtil.clamp(position.getRadians(), WristConstants.kMinWristAngleRadians, WristConstants.kMaxWristAngleRadians))
            + m_feedforwardController.calculate(inputs.angleRadians, m_profiledPIDController.getSetpoint().velocity)
        );
    }

    /** Returns the angle of the wrist in radians */
    public double getAngleRadians() {
        return inputs.angleRadians;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.angleRadians);
    }

    public void stop() {
        setVoltage(0);
    }

    /** Sets the feedforward kG value for the wrist */
    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }

    /** Returns a command that sets the wrist at the desired angle 
     * ENDS WHEN ANGLE IS REACHED
     * @param angle The desired angle of the wrist
    */
    public Command applyAngle(Rotation2d angle) {
        return new FunctionalCommand(
            () -> m_profiledPIDController.reset(getAngleRadians()),
            () -> setAngle(angle), 
            (interrupted) -> {},
            // () -> {io.resetController();},
            // () -> setPosition(angle), 
            // (interrupted) -> {holdAngle(angle);},
            () -> Math.abs(angle.minus(getAngle()).getRadians()) < WristConstants.wristAngleToleranceRadians,
            this
        );
    }

    /** Returns a command that sets the wrist at the desired angle 
     * CONTINUES UNTIL INTERUPTED EXTERNALLY
     * @param angle The desired angle of the wrist
    */
    public Command holdAngle(Rotation2d angle){
        return new FunctionalCommand(
            () -> m_profiledPIDController.reset(getAngleRadians()),
            () -> setAngle(angle), 
            (interrupted) -> {},
            () -> {return false;},
            this
        );
    }

    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * WristConstants.maxOperatorControlVolts));
    }

    @Override
    public void periodic() {
        //System.out.println("Wrist Angle: " + this.getAngleRadians());
        io.updateInputs(inputs);
        Logger.recordOutput("Wrist/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.processInputs("Wrist", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command wristSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command wristSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }
}
