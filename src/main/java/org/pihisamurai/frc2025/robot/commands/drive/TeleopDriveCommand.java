package org.pihisamurai.frc2025.robot.commands.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PointOfInterest;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PoseOfInterest;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;
import org.pihisamurai.lib.utils.TriFunction;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.DriverControlConstants.doubleClutchRotationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.DriverControlConstants.doubleClutchTranslationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.DriverControlConstants.maxRotationSpeedRadPerSec;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.DriverControlConstants.maxTranslationSpeedMPS;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.DriverControlConstants.singleClutchRotationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.DriverControlConstants.singleClutchTranslationFactor;

public class TeleopDriveCommand extends Command {
    //The raw speed suppliers, unaffected by the clutches. A reference to these is maintained in order to make applying and unapplying clutches easier
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawOmegaSupplier;

    //The required drive subsystem
    private final DriveSubsystem m_drive;

    //The clutch factors being used
    private double transClutch = 1.0;
    private double rotClutch = 1.0;

    //The actual speed suppliers, these are affected by the clutch factor
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    //Request Generator declarations

    //The request generator is a function that takes three doubles as parameters and returns a SwerveRequest.
    //During every loop, the requestGenerator is invoked with the values of the three doublesuppliers as parameters,
    //And the resulting SwerveRequest is applied to the Drivetrain
    private TriFunction<Double,Double,Double,SwerveRequest> requestGenerator;

    //A swerve request override. If this optional is not empty, it signals to the TeleopDriveCommand that the request generator has been overridden, and will not
    //generate a default requestGenerator during reloadCommand()
    //The presence of a requestGeneratorOverride supersedes the presence of a headingSupplier
    private Optional<TriFunction<Double,Double,Double,SwerveRequest>> requestGeneratorOverride = Optional.empty();

    //A heading override. If this optional is not empty, it signals to the TeleopDriveCommand that the heading has been locked,
    //And will generate a requestGenerator that returns a FieldCentricFacingAngle request
    private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();

    public TeleopDriveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        m_drive = drive;
        rawXSupplier = () -> xSupplier.getAsDouble() * maxTranslationSpeedMPS;
        rawYSupplier = () -> ySupplier.getAsDouble() * maxTranslationSpeedMPS;
        rawOmegaSupplier = () -> omegaSupplier.getAsDouble() * maxRotationSpeedRadPerSec;
        addRequirements(drive);
    }

    /* ######################################################################## */
    /* # Standard methods inherited from Command                              # */
    /* ######################################################################## */
    @Override
    public void initialize() {
        reloadCommand();
    }

    @Override
    public void execute(){
        m_drive.acceptRequest(
            requestGenerator.apply(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                omegaSupplier.getAsDouble()
            )
        );
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTeleopEnabled();
    }

    
    /* ######################################################################## */
    /* # Internal Utility Methods                                             # */
    /* ######################################################################## */

    /**
     * Reloads the command after a state change. Every time the command's state changes, this function MUST
     * be called for the changes to come into effect
     */
    private void reloadCommand() {
        xSupplier = () -> rawXSupplier.getAsDouble() * transClutch;
        ySupplier = () -> rawYSupplier.getAsDouble() * transClutch;
        omegaSupplier = () -> rawOmegaSupplier.getAsDouble() * rotClutch;
        if (requestGeneratorOverride.isPresent()){
            requestGenerator = requestGeneratorOverride.get();
        } else if (headingSupplier.isPresent()) {
            requestGenerator = (vx,vy,omega) -> {
                FieldCentricFacingAngle request = new FieldCentricFacingAngle()
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withTargetDirection(headingSupplier.get().get())
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
                request.HeadingController.setPID(5, 0, 0);
                request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
                return request;
            };
        } else {
            requestGenerator = (vx,vy,omega) -> {
                return new ApplyFieldSpeeds()
                    .withSpeeds(new ChassisSpeeds(vx,vy,omega))
                    .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
            };
        }
    }

    /** Overrides the requestGenerator */
    private void overrideRequestGenerator(TriFunction<Double,Double,Double,SwerveRequest> newRequestGenerator){
        this.requestGeneratorOverride = Optional.of(newRequestGenerator);
        reloadCommand();
    }

    /** Clears any requestGenerator overrides */
    private void clearRequestGeneratorOverride() {
        this.requestGeneratorOverride = Optional.empty();
        reloadCommand();
    }

    /** Sets an arbitrary clutch factor */
    private void setClutchFactor(double transClutch, double rotClutch) {
        this.transClutch = transClutch;
        this.rotClutch = rotClutch;
        reloadCommand();
    }

    /** Sets an arbitrary heading lock, based on a headingSupplier */
    private void setHeadingLock(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = Optional.of(headingSupplier);
        reloadCommand();
    }

    /** Removes any heading lock */
    private void clearHeadingLock() {
        this.headingSupplier = Optional.empty();
        reloadCommand();
    }

    /** Returns whether or not the request generator has been overridden */
    public boolean requestGeneratorOverridden() {
        return requestGeneratorOverride.isPresent();
    }

    
    /* ######################################################################## */
    /* # Public Command Factories                                             # */
    /* ######################################################################## */


    /** Returns a command that applies an arbitrary clutch factor */
    public Command applyClutchFactor(double transClutch, double rotClutch) {
        return Commands.startEnd(
            () -> setClutchFactor(transClutch, rotClutch),
            () -> setClutchFactor(1.0, 1.0)
        );
    }

    /** Returns a command that applies an arbitrary request generator override */
    public Command applyRequestGeneratorOverride(TriFunction<Double,Double,Double,SwerveRequest> override) {
        return Commands.startEnd(
            () -> overrideRequestGenerator(override), 
            () -> clearRequestGeneratorOverride()
        );
    }

    /** Returns a command that applies an arbitrary heading lock, based on a headingSupplier */
    public Command applyHeadingLock(Supplier<Rotation2d> headingSupplier) {
        return Commands.startEnd(
            () -> setHeadingLock(headingSupplier),
            () -> clearHeadingLock()
        );
    }

    /** Returns a command that applies an arbitrary heading lock, based on a static heading */
    public Command applyHeadingLock(Rotation2d heading) {
        return applyHeadingLock(() -> heading);
    }

    /** Returns a command that applies a single clutch to the TeleopDriveCommand */
    public Command applySingleClutch(){
        return applyClutchFactor(singleClutchTranslationFactor, singleClutchRotationFactor);
    }

    /** Returns a command that applies a double clutch to the TeleopDriveCommand */
    public Command applyDoubleClutch(){
        return applyClutchFactor(doubleClutchTranslationFactor, doubleClutchRotationFactor);
    }

    /** Returns a command that applies a reef-oriented heading lock */
    public Command applyReefHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue 
                ? () -> {
                    Translation2d teamReef = PointOfInterest.BLU_REEF.position;
                    Rotation2d angleToReef = teamReef.minus(m_drive.getPose().getTranslation()).getAngle();
                    return angleToReef;
                }
                : () -> {
                    Translation2d teamReef = PointOfInterest.RED_REEF.position;
                    Rotation2d angleToReef = teamReef.minus(m_drive.getPose().getTranslation()).getAngle();
                    return angleToReef;
                }
        );
    }

    /** Returns a command that applies a Processor side coral station-oriented heading lock*/
    public Command applyProcessorCoralHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
                ? PoseOfInterest.BLU_CORAL_STATION_PROCESSOR.pose.getRotation()
                : PoseOfInterest.RED_CORAL_STATION_PROCESSOR.pose.getRotation()
        );
    }

    /** Returns a command that applies an Opposite side coral station-oriented heading lock */
    public Command applyOppositeCoralHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
                ? PoseOfInterest.BLU_CORAL_STATION_OPPOSITE.pose.getRotation()
                : PoseOfInterest.RED_CORAL_STATION_OPPOSITE.pose.getRotation()
        );
    }

    /** Returns a command that applies a forward-oriented heading lock */
    public Command applyForwardHeadingLock() {
        return applyHeadingLock(Rotation2d.fromDegrees(0));
    }

}
