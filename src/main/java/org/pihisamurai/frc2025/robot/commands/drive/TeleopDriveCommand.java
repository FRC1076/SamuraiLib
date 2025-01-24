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
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawOmegaSupplier;
    private final DriveSubsystem m_drive;
    private double transClutch = 1.0;
    private double rotClutch = 1.0;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;
    private TriFunction<Double,Double,Double,SwerveRequest> requestGenerator;
    private Optional<TriFunction<Double,Double,Double,SwerveRequest>> requestGeneratorOverride;
    private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
    public TeleopDriveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        m_drive = drive;
        rawXSupplier = () -> xSupplier.getAsDouble() * maxTranslationSpeedMPS;
        rawYSupplier = () -> ySupplier.getAsDouble() * maxTranslationSpeedMPS;
        rawOmegaSupplier = () -> omegaSupplier.getAsDouble() * maxRotationSpeedRadPerSec;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        reloadCommand();
    }

    /*Reloads the command*/
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
                request.HeadingController.setPID(3.5, 0, 0);
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

    private void overrideRequestGenerator(TriFunction<Double,Double,Double,SwerveRequest> newRequestGenerator){
        this.requestGeneratorOverride = Optional.of(newRequestGenerator);
        reloadCommand();
    }

    private void clearRequestGeneratorOverride() {
        this.requestGeneratorOverride = Optional.empty();
        reloadCommand();
    }

    private void setClutchFactor(double transClutch, double rotClutch) {
        this.transClutch = transClutch;
        this.rotClutch = rotClutch;
        reloadCommand();
    }

    private void setHeadingLock(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = Optional.of(headingSupplier);
        reloadCommand();
    }

    private void clearHeadingLock() {
        this.headingSupplier = Optional.empty();
        reloadCommand();
    }

    public boolean requestGeneratorOverridden() {
        return requestGeneratorOverride.isPresent();
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

    public Command applyClutchFactor(double transClutch, double rotClutch) {
        return Commands.startEnd(
            () -> setClutchFactor(transClutch, rotClutch),
            () -> setClutchFactor(1.0, 1.0)
        );
    }

    public Command applyHeadingLock(Supplier<Rotation2d> headingSupplier) {
        return Commands.startEnd(
            () -> setHeadingLock(headingSupplier),
            () -> clearHeadingLock()
        );
    }

    public Command applyRequestGeneratorOverride(TriFunction<Double,Double,Double,SwerveRequest> override) {
        return Commands.startEnd(
            () -> overrideRequestGenerator(override), 
            () -> clearRequestGeneratorOverride()
        );
    }

    public Command applyHeadingLock(Rotation2d heading) {
        return applyHeadingLock(() -> heading);
    }

    /** Returns a command that applies a single clutch to the TeleopDriveCommand */
    public Command applySingleClutch(){
        return applyClutchFactor(singleClutchTranslationFactor, singleClutchRotationFactor);
    }

    //** Returns a command that applies a double clutch to the TeleopDriveCommand */
    public Command applyDoubleClutch(){
        return applyClutchFactor(doubleClutchTranslationFactor, doubleClutchRotationFactor);
    }

    public Command applyReefHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue 
                ? () -> {
                    Translation2d teamReef = PointOfInterest.BLU_REEF.position;
                    Rotation2d angleToReef = teamReef.minus(m_drive.getPose().getTranslation()).getAngle();
                    return angleToReef.rotateBy(Rotation2d.fromDegrees(180));
                }
                : () -> {
                    Translation2d teamReef = PointOfInterest.RED_REEF.position;
                    Rotation2d angleToReef = teamReef.minus(m_drive.getPose().getTranslation()).getAngle();
                    return angleToReef.rotateBy(Rotation2d.fromDegrees(180));
                }
        );
    }

    public Command applyProcessorCoralHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue
                ? PoseOfInterest.BLU_CORAL_STATION_PROCESSOR.pose.getRotation()
                : PoseOfInterest.RED_CORAL_STATION_PROCESSOR.pose.getRotation()
        );
    }

    //** Returns a command that applies a heading lock oriented to the opposite-side coral station to the TeleopDriveCommand */
    public Command applyOppositeCoralHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue
                ? PoseOfInterest.BLU_CORAL_STATION_OPPOSITE.pose.getRotation()
                : PoseOfInterest.RED_CORAL_STATION_OPPOSITE.pose.getRotation()
        );
    }

    //** Returns a command that applies a heading lock oriented directly forward to the TeleopDriveCommand */
    public Command applyForwardHeadingLock() {
        return applyHeadingLock(Rotation2d.fromDegrees(0));
    }

}
