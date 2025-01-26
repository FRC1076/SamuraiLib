// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Akit;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.DriveClosedLoopTeleop;
import frc.robot.commands.drive.DirectDriveToNearestBranch;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.SuperstructureVisualizer;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_drive;
  private final ElevatorSubsystem m_elevator;
  private final WristSubsystem m_wrist;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
    
  private final SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /* 
    DO NOT REFACTOR INTO A SWITCH STATEMENT!!! 
    Because the expressions being evaluated are known at compile time, the compiler will 
    discard the unused branches, functioning as a shitty bootleg version of
    if constexpr

    Also, ignore the "comparing identical expressions" and "dead code" warnings
    */

    if (Akit.currentMode == 0) {
        m_drive = new DriveSubsystem(new DriveIOHardware(TunerConstants.createDrivetrain()));
    } else if (Akit.currentMode == 1) {
        m_drive = new DriveSubsystem(new DriveIOSim(TunerConstants.createDrivetrain()));
        SuperstructureVisualizer superstructureVisualizer = new SuperstructureVisualizer();
        m_elevator = new ElevatorSubsystem(new ElevatorIOSim(superstructureVisualizer.getElevatorLigament()));
        m_wrist = new WristSubsystem(new WristIOSim(superstructureVisualizer.getWristLigament()));
    }

    m_drive.setDefaultCommand(
      new DriveClosedLoopTeleop(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kControllerDeadband), 
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kControllerDeadband),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kControllerDeadband), 
        m_drive)
    );

    m_elevator.setDefaultCommand(new RunCommand(
      () -> m_elevator.setVoltage(0),
      m_elevator)
    );

    m_wrist.setDefaultCommand(new RunCommand(
      () -> m_wrist.setVoltage(0),
      m_wrist)
    );

    // Configure the trigger bindings
    configureBindings();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //m_driverController.leftTrigger(0.7).whileTrue(new DirectDriveToNearestBranch(m_drive, true));
    //m_driverController.rightTrigger(0.7).whileTrue(new DirectDriveToNearestBranch(m_drive, false));
    
    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> m_wrist.setPosition(Rotation2d.fromDegrees(-23.5).getRadians()), m_wrist),
      new RunCommand(() -> m_elevator.setPosition(0.08128), m_elevator)
    ));

    m_driverController.rightTrigger().whileFalse(new ParallelCommandGroup(
      new RunCommand(() -> m_wrist.setPosition(Rotation2d.fromDegrees(90).getRadians()), m_wrist),
      new RunCommand(() -> m_elevator.setPosition(0), m_elevator)
    ));

    m_driverController.y().whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> m_wrist.setPosition(Rotation2d.fromDegrees(-45).getRadians()), m_wrist),
      new RunCommand(() -> m_elevator.setPosition(1.8161), m_elevator)
    ));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
