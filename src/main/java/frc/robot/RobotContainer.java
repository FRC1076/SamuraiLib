// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.commands.Autos;
import frc.robot.commands.drive.DirectDriveToPoseCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberIOHardware;
import frc.robot.subsystems.grabber.GrabberIOSim;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexIOHardware;
import frc.robot.subsystems.index.IndexIOSim;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.Akit;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final Trigger m_indexBeamBreak;
    private final Trigger m_transferBeamBreak;
    private final Trigger m_grabberBeamBreak;
    private final Trigger m_interruptElevator;
    private final Trigger m_interruptGrabber;
    private final Superstructure m_superstructure;
    private final SuperstructureVisualizer superVis;


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);

    private final CommandXboxController m_operatorController =
        new CommandXboxController(OIConstants.kOperatorControllerPort);
    
    private final SendableChooser<Command> m_autoChooser;

    private final TeleopDriveCommand teleopDriveCommand;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

    /* 
    DO NOT REFACTOR INTO A SWITCH STATEMENT!!! 
    Because the expressions being evaluated are known at compile time, the compiler will 
    discard the unused branches, functioning as a shitty bootleg version of
    if constexpr
    Also, ignore the "comparing identical expressions" and "dead code" warnings
    */
    
    
        m_indexBeamBreak = new Trigger(new DigitalInput(BeamBreakConstants.indexBeamBreakPort)::get);
        m_transferBeamBreak = new Trigger(new DigitalInput(BeamBreakConstants.transferBeamBreakPort)::get);
        m_grabberBeamBreak = new Trigger(new DigitalInput(BeamBreakConstants.grabberBeamBreakPort)::get);
        m_interruptElevator = new Trigger(() -> m_operatorController.getLeftY() != 0);
        m_interruptGrabber = new Trigger(() -> m_operatorController.getRightY() != 0);


        if (Akit.currentMode == 0) {
            m_drive = new DriveSubsystem(new DriveIOHardware(TunerConstants.createDrivetrain()));
            m_elevator = new ElevatorSubsystem(new ElevatorIOHardware());
            m_wrist = new WristSubsystem(new WristIOHardware());
            m_grabber = new GrabberSubsystem(new GrabberIOHardware());
            m_index = new IndexSubsystem(new IndexIOHardware());
        } else if (Akit.currentMode == 1) {
            m_drive = new DriveSubsystem(new DriveIOSim(TunerConstants.createDrivetrain()));
            m_elevator = new ElevatorSubsystem(new ElevatorIOSim());
            m_wrist = new WristSubsystem(new WristIOSim());
            m_grabber = new GrabberSubsystem(new GrabberIOSim());
            m_index = new IndexSubsystem(new IndexIOSim());
        }

        m_superstructure = new Superstructure(
            m_elevator,
            m_grabber,
            m_index, 
            m_wrist, 
            m_indexBeamBreak, 
            m_transferBeamBreak, 
            m_grabberBeamBreak
        );

        superVis = new SuperstructureVisualizer(m_superstructure);

        teleopDriveCommand = m_drive.CommandBuilder.teleopDrive(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kControllerDeadband), 
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kControllerDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.kControllerDeadband)
        );

        m_drive.setDefaultCommand(
            teleopDriveCommand
        );

        m_wrist.setDefaultCommand(m_wrist.applyManualControl(
            () -> MathUtil.applyDeadband(-m_operatorController.getRightY(), OIConstants.kControllerDeadband)
        ));

        m_elevator.setDefaultCommand(m_elevator.applyManualControl(
            () -> MathUtil.applyDeadband(-m_operatorController.getLeftY(), OIConstants.kControllerDeadband)
        ));

        // Configure the trigger bindings
        configureDriverBindings();

        // Configure the operator bindings
        configureOperatorBindings();
    
        //configure beam break triggers
        configureBeamBreakTriggers();

        //Build the auto chooser with PathPlanner
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
        
    }

    private void configureDriverBindings() {

        m_driverController.leftTrigger().whileTrue(
            m_drive.CommandBuilder.directDriveToNearestLeftBranch()
        );

        m_driverController.rightTrigger().whileTrue(
            m_drive.CommandBuilder.directDriveToNearestRightBranch()
        );

        //Point to reef
        m_driverController.a().whileTrue(teleopDriveCommand.applyReefHeadingLock());

        //Apply single clutch
        m_driverController.rightBumper().whileTrue(teleopDriveCommand.applySingleClutch());

        //Apply double clutch
        m_driverController.leftBumper().whileTrue(teleopDriveCommand.applyDoubleClutch());

        m_driverController.x().and(
            m_driverController.leftBumper().and(
                m_driverController.rightBumper()
            ).negate()
        ).whileTrue(teleopDriveCommand.applyLeftStationHeadingLock());

        m_driverController.b().whileTrue(teleopDriveCommand.applyRightStationHeadingLock());

        m_driverController.y().whileTrue(teleopDriveCommand.applyForwardHeadingLock());

        m_driverController.leftBumper().and(
            m_driverController.rightBumper().and(
                m_driverController.x()
            )
        ).onTrue(new InstantCommand(
            () -> m_drive.resetHeading()
        ));

        //Quasistsic and Dynamic control scheme for Elevator Sysid
        m_driverController.rightBumper().and(
          m_driverController.a()
        ).onTrue(m_elevator.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kForward));

        m_driverController.rightBumper().and(
          m_driverController.b()
        ).onTrue(m_elevator.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        m_driverController.rightBumper().and(
          m_driverController.x()
        ).onTrue(m_elevator.elevatorSysIdDynamic(SysIdRoutine.Direction.kForward));
        
        m_driverController.rightBumper().and(
          m_driverController.y()
        ).onTrue(m_elevator.elevatorSysIdDynamic(SysIdRoutine.Direction.kReverse));

        //Quasistsic and Dynamic control scheme for Wrist Sysid
        // m_driverController.rightBumper().and(
        //   m_driverController.a()
        // ).onTrue(m_wrist.wristSysIdQuasistatic(SysIdRoutine.Direction.kForward));

        // m_driverController.rightBumper().and(
        //   m_driverController.b()
        // ).onTrue(m_wrist.wristSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // m_driverController.rightBumper().and(
        //   m_driverController.x()
        // ).onTrue(m_wrist.wristSysIdDynamic(SysIdRoutine.Direction.kForward));
        
        // m_driverController.rightBumper().and(
        //   m_driverController.y()
        // ).onTrue(m_wrist.wristSysIdDynamic(SysIdRoutine.Direction.kReverse));


    }

    private void configureOperatorBindings() {
        
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();
        
        //L1
        m_operatorController.x().onTrue(superstructureCommands.preL1());

        //L2
        m_operatorController.a().onTrue(superstructureCommands.preL2());

        //L3
        m_operatorController.b().onTrue(superstructureCommands.preL3());

        //L4
        m_operatorController.y().onTrue(superstructureCommands.preL4());

        //Net
        m_operatorController.y().and(m_operatorController.leftBumper()).onTrue(superstructureCommands.preNet());

        //High Intake
        m_operatorController.b().and(m_operatorController.leftBumper()).onTrue(superstructureCommands.highAlgaeIntake());

        //Low Intake
        m_operatorController.a().and(m_operatorController.leftBumper()).onTrue(superstructureCommands.lowAlgaeIntake());

        //Processor
        m_operatorController.x().and(m_operatorController.leftBumper()).onTrue(superstructureCommands.preProcessor());

        //Retract mechanisms and stop grabber
        m_operatorController.rightTrigger().onFalse(superstructureCommands.stopAndRetract());

    }

    private void configureBeamBreakTriggers() {
        m_indexBeamBreak.or(
            m_transferBeamBreak.or(
                m_grabberBeamBreak
            )
        ).onChange(
            m_superstructure.CommandBuilder.calculatePossession()
        );
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
