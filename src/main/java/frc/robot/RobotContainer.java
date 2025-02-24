// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drive.DirectDriveToPoseCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
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
import frc.robot.subsystems.led.LEDIODigitalPins;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import lib.extendedcommands.CommandUtils;
import lib.hardware.hid.SamuraiXboxController;
import lib.vision.PhotonVisionLocalizer;
import lib.vision.VisionLocalizationSystem;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.subsystems.Superstructure;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultSingleTagStdDevs;
import static frc.robot.Constants.VisionConstants.Photonvision.driverCamName;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultMultiTagStdDevs;
import static frc.robot.Constants.VisionConstants.fieldLayout;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
    private final Trigger m_interruptElevator;
    private final Trigger m_interruptWrist;
    private final Superstructure m_superstructure;
    private final SuperstructureVisualizer superVis;
    private final Elastic m_elastic;
    private final LEDSubsystem m_LEDs;


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);

    private final CommandXboxController m_operatorController =
        new SamuraiXboxController(OIConstants.kOperatorControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);
    /* 
    private final CommandXboxController m_beamBreakController = 
        new CommandXboxController(2);
    */
    
    private final SendableChooser<Command> m_autoChooser;

    private final VisionLocalizationSystem m_vision = new VisionLocalizationSystem();

    private final VisionSystemSim m_visionSim;

   // private final PhotonCamera m_driveCamera;

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
        DigitalInput indexDIO = new DigitalInput(BeamBreakConstants.indexBeamBreakPort);
        DigitalInput transferDIO = new DigitalInput(BeamBreakConstants.transferBeamBreakPort);
        m_indexBeamBreak = new Trigger(() -> {return ! indexDIO.get();});//.or(m_beamBreakController.a());
        m_transferBeamBreak = new Trigger(() -> {return ! transferDIO.get();});//.or(m_beamBreakController.x());
        m_interruptElevator = new Trigger(() -> m_operatorController.getLeftY() != 0);
        m_interruptWrist = new Trigger(() -> m_operatorController.getRightY() != 0);

        // m_driveCamera = new PhotonCamera(driverCamName);
        // m_driveCamera.setDriverMode(true);
        
        if (SystemConstants.currentMode == 0) {
            m_visionSim = null;
            m_drive = new DriveSubsystem(new DriveIOHardware(TunerConstants.createDrivetrain()), m_vision);
            m_elevator = new ElevatorSubsystem(new ElevatorIOHardware());
            m_wrist = new WristSubsystem(new WristIOHardware());
            m_grabber = new GrabberSubsystem(new GrabberIOHardware());
            m_index = new IndexSubsystem(new IndexIOHardware());
            m_elastic = new Elastic();
            m_LEDs = new LEDSubsystem(new LEDIODigitalPins());
            for (PhotonConfig config : PhotonConfig.values()){
                var cam = new PhotonCamera(config.name);
                m_vision.addCamera(new PhotonVisionLocalizer(
                    cam, 
                    config.offset,
                    config.multiTagPoseStrategy,
                    config.singleTagPoseStrategy,
                    () -> m_drive.getPose().getRotation(),
                    fieldLayout,
                    config.defaultSingleTagStdDevs, 
                    config.defaultMultiTagStdDevs)
                );
            }
        } else if (SystemConstants.currentMode == 1) {
           
            
            m_drive = new DriveSubsystem(new DriveIOSim(TunerConstants.createDrivetrain()), m_vision);
            m_elevator = new ElevatorSubsystem(new ElevatorIOSim());
            m_wrist = new WristSubsystem(new WristIOSim());
            m_grabber = new GrabberSubsystem(new GrabberIOSim());
            m_index = new IndexSubsystem(new IndexIOSim());
            m_elastic = new Elastic();
            m_LEDs = new LEDSubsystem(new LEDIOSim());
            m_visionSim = new VisionSystemSim("main");
            for (PhotonConfig config : PhotonConfig.values()){
                var cam = new PhotonCamera(config.name);
                m_vision.addCamera(new PhotonVisionLocalizer(
                    cam, 
                    config.offset,
                    config.multiTagPoseStrategy,
                    config.singleTagPoseStrategy,
                    () -> m_drive.getPose().getRotation(),
                    fieldLayout,
                    kDefaultSingleTagStdDevs, 
                    kDefaultMultiTagStdDevs)
                );
                m_visionSim.addCamera(new PhotonCameraSim(cam),config.offset);
            }
            CommandUtils.makePeriodic(() -> m_visionSim.update(m_drive.getPose()));
        }

        m_superstructure = new Superstructure(
            m_elevator,
            m_grabber,
            m_index, 
            m_wrist, 
            m_elastic,
            m_LEDs,
            m_indexBeamBreak, 
            m_transferBeamBreak, 
            () -> false
        );

        superVis = new SuperstructureVisualizer(m_superstructure);

        teleopDriveCommand = m_drive.CommandBuilder.teleopDrive(
            () -> -m_driverController.getLeftY(), 
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        );

        m_drive.setDefaultCommand(
            teleopDriveCommand
        );

        m_wrist.setDefaultCommand(m_wrist.applyManualControl(
            () -> -m_operatorController.getRightY()
        ));

        m_elevator.setDefaultCommand(m_elevator.applyManualControl(
            () -> -m_operatorController.getLeftY()
        ));

        configureNamedCommands();

        // Configure the trigger bindings
        configureDriverBindings();

        // Configure the operator bindings
        configureOperatorBindings();
    
        //configure beam break triggers
        configureBeamBreakTriggers();

        //Build the auto chooser with PathPlanner
        m_autoChooser = AutoBuilder.buildAutoChooser();
        m_autoChooser.addOption(
            "DoNothingBlue", 
            Commands.runOnce(() -> m_drive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))))
        );
        m_autoChooser.addOption(
            "DoNothingRed", 
            Commands.runOnce(() -> m_drive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180))))
        );
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

    private void configureNamedCommands(){
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();
        NamedCommands.registerCommand("preL1", superstructureCommands.preL1());
        NamedCommands.registerCommand("preL2", superstructureCommands.preL2());
        NamedCommands.registerCommand("preL3", superstructureCommands.preL3());
        NamedCommands.registerCommand("preL4", superstructureCommands.preL4());
        // TODO: Change name to match later
        NamedCommands.registerCommand("scoreGamePiece", superstructureCommands.doGrabberAction());
        NamedCommands.registerCommand("stopAndRetract", superstructureCommands.stopAndRetract());
    }

    private void configureDriverBindings() {
        /*
        
        m_driverController.leftTrigger().whileTrue(
            m_drive.CommandBuilder.directDriveToNearestLeftBranch()
        );

        m_driverController.rightTrigger().whileTrue(
            m_drive.CommandBuilder.directDriveToNearestRightBranch()
        );
        
        */

        // Point to reef
        m_driverController.a().whileTrue(teleopDriveCommand.applyReefHeadingLock());

        // Apply single clutch
        m_driverController.rightBumper().and(m_driverController.leftBumper().negate())
            .whileTrue(teleopDriveCommand.applySingleClutch());

        // Apply double clutch
        m_driverController.leftBumper().and(m_driverController.rightBumper().negate())
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        // Apply FPV Driving TODO: Finalize bindings and FPV clutch with drive team
        m_driverController.leftBumper().and(m_driverController.rightBumper())
            .whileTrue(teleopDriveCommand.applyFPVDrive());

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
    }

    private void configureOperatorBindings() {

        /*
        if (SystemConstants.operatorSysID) {
            
            // Quasistsic and Dynamic control scheme for Elevator Sysid
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
            m_driverController.rightBumper().and(
                m_driverController.a()
            ).onTrue(m_wrist.wristSysIdQuasistatic(SysIdRoutine.Direction.kForward));

            m_driverController.rightBumper().and(
                m_driverController.b()
            ).onTrue(m_wrist.wristSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            m_driverController.rightBumper().and(
                m_driverController.x()
            ).onTrue(m_wrist.wristSysIdDynamic(SysIdRoutine.Direction.kForward));
        
            m_driverController.rightBumper().and(
                m_driverController.y()
            ).onTrue(m_wrist.wristSysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
        */
        
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandBuilder();
        
        // L1
        m_operatorController.x()
        .and(m_operatorController.leftBumper().negate())
            .whileTrue(superstructureCommands.preL1());

        // L2
        m_operatorController.a()
        .and(m_operatorController.leftBumper().negate())
            .whileTrue(superstructureCommands.preL2());

        // L3
        m_operatorController.b()
            .and(m_operatorController.leftBumper().negate())
            .whileTrue(superstructureCommands.preL3());

        // L4
        m_operatorController.y()
        .and(m_operatorController.leftBumper().negate())
            .whileTrue(superstructureCommands.preL4());

        // Processor
        m_operatorController.x()
            .and(m_operatorController.leftBumper())
            .whileTrue(superstructureCommands.preProcessor());

        // Low Algae Intake
        m_operatorController.a()
            .and(m_operatorController.leftBumper())
            .whileTrue(superstructureCommands.lowAlgaeIntake());

        // High Algae Intake
        m_operatorController.b()
            .and(m_operatorController.leftBumper())
            .whileTrue(superstructureCommands.highAlgaeIntake());

        // Net
        m_operatorController.y()
            .and(m_operatorController.leftBumper())
            .whileTrue(superstructureCommands.preNet());

        // Set default command for Indexer to continuously run
        //m_index.setDefaultCommand(superstructureCommands.indexCoral());

        // Coral Intake and transfer into Grabber
        m_operatorController.leftTrigger()
            .and(m_operatorController.leftBumper().negate())
            .whileTrue(superstructureCommands.intakeCoral())
            .whileFalse(superstructureCommands.stopIntake());

        // Ground Algae Intake
        m_operatorController.leftTrigger().and(m_operatorController.leftBumper()).whileTrue(superstructureCommands.groundAlgaeIntake());

        // Does Grabber action, ie. outtake coral/algae depending 
        m_operatorController.rightTrigger().whileTrue(superstructureCommands.doGrabberAction());

        // Retract mechanisms and stop grabber
        m_operatorController.rightTrigger().whileFalse(superstructureCommands.stopAndRetract());

        m_interruptElevator.onTrue(superstructureCommands.interruptElevator());

        m_interruptWrist.onTrue(superstructureCommands.interruptWrist());
    }

    private void configureBeamBreakTriggers() {

        m_indexBeamBreak.onChange(
            m_superstructure.CommandBuilder.updatePossessionAndKg()
        );

        m_transferBeamBreak.onChange(
            m_superstructure.CommandBuilder.updatePossessionAndKg()
        );
        
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        //return AutoBuilder.buildAuto("J4_K4_L4_A4");
        return m_autoChooser.getSelected();
    }
}
