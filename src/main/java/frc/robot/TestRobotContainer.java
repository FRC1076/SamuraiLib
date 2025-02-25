package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.samuraidrive.SamuraiSwerveDrivetrain;
import lib.hardware.hid.SamuraiXboxController;

public class TestRobotContainer {
    private final SamuraiSwerveDrivetrain m_drive = new SamuraiSwerveDrivetrain();
    private final CommandXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);
    
    public TestRobotContainer() {
        m_drive.setDefaultCommand(
            m_drive.getTeleopDriveCommand(
                m_driverController::getLeftX,
                m_driverController::getLeftY,
                m_driverController::getRightX
            )
        );
    }
}
