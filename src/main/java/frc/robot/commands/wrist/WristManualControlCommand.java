// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WristManualControlCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DoubleSupplier m_controlSupplier;
  private final WristSubsystem m_wrist;

  /**
   * Creates a new WristManualControlCommand.
   *
   * @param controlSupplier The DoubleSupplier used to control the wrist
   * @param wrist The wrist subsystem used by this command
   */
  public WristManualControlCommand(DoubleSupplier controlSupplier, WristSubsystem wrist) {
    m_wrist = wrist;
    m_controlSupplier = controlSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setVoltage(m_controlSupplier.getAsDouble() * WristConstants.maxOperatorControlVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
