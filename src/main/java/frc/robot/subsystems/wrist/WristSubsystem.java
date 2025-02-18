// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final SysIdRoutine sysid = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, Volts.of(1), null,
            (state) -> Logger.recordOutput("Wrist/SysIDState", state.toString())
        ), 
        new SysIdRoutine.Mechanism(
            (voltage) -> setVoltageCharacterization(voltage.in(Volts)),
            null,
            this
        )
    );

    public WristSubsystem(WristIO io) {
        this.io = io;
    }
    
    /** Sets the voltage of the wrist motors*/
    public void setVoltage(double volts) {
        
        if (this.getAngleRadians() > WristConstants.kMaxWristAngleRadians && volts > 0) {
            volts = 0;
        } else if (this.getAngleRadians() < WristConstants.kMinWristAngleRadians && volts < 0) {
            volts = 0;
        }


        io.setVoltage(volts);
    }

    private void setVoltageCharacterization(double volts) {
        io.setVoltageCharacterization(volts);
    }
    /** Sets the desired rotation of the wrist */
    public void setPosition(Rotation2d position) {
        io.setPosition(MathUtil.clamp(position.getRadians(), WristConstants.kMinWristAngleRadians, WristConstants.kMaxWristAngleRadians));
        //io.setPosition(position.getRadians());
    }

    /** Returns the angle of the wrist in degrees */
    public Rotation2d getAngle(){
        return inputs.angle;
    }

    /** Returns the angle of the wrist in radians */
    public double getAngleRadians() {
        return getAngle().getRadians();
    }

    public void stop() {
        setVoltage(0);
    }

    /** Sets the feedforward kG value for the wrist */
    public void setKg(double kg) {
        this.io.setFFkG(kg);
    }

    /** Returns a command that sets the wrist at the desired angle 
     * ENDS WHEN ANGLE IS REACHED
     * @param angle The desired angle of the wrist
    */
    public Command applyAngle(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {io.resetController();},
            () -> setPosition(angle), 
            (interrupted) -> {holdAngle(angle);},
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
            () -> {io.resetController();},
            () -> setPosition(angle), 
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
