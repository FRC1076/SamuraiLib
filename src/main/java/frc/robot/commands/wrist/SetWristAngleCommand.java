// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetWristAngleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Rotation2d targetAngle;
  private final WristSubsystem m_wrist;

  /**
   * Creates a new SetWristAngleCommand.
   *
   * @param targetAngle The target angle.
   * @param wrist The wrist subsystem used by this command
   */
  public SetWristAngleCommand(Rotation2d targetAngle, WristSubsystem wrist) {
    m_wrist = wrist;
    this.targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setPosition(targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle.minus(m_wrist.getAngle()).getRadians()) > WristConstants.wristAngleToleranceRadians;
  }
}
