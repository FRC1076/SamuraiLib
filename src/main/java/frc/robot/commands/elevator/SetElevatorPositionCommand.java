// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetElevatorPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final double targetPosition;
  private final ElevatorSubsystem m_elevator;

  /**
   * Creates a new SetElevatorPositionCommand.
   *
   * @param targetPosition The targetPosition of the elevator (in meters).
   * @param elevator The elevator subsystem used by this command
   */
  public SetElevatorPositionCommand(double targetPosition, ElevatorSubsystem elevator) {
    m_elevator = elevator;
    this.targetPosition = targetPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetPosition - m_elevator.getPositionMeters()) > ElevatorConstants.elevatorPositionToleranceMeters;
  }
}
