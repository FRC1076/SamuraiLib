package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged rearLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged rearRightInputs = new ModuleIOInputsAutoLogged();

    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }
}
