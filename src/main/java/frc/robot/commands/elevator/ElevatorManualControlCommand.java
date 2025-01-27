// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorManualControlCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DoubleSupplier m_controlSupplier;
  private final ElevatorSubsystem m_elevator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorManualControlCommand(DoubleSupplier controlSupplier, ElevatorSubsystem elevator) {
    m_elevator = elevator;
    m_controlSupplier = controlSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setVoltage(m_controlSupplier.getAsDouble() * ElevatorConstants.maxOperatorControlVolts);
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
