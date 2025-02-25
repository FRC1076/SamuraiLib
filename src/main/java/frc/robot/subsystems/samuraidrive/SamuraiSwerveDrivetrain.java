package frc.robot.subsystems.samuraidrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.hardware.swerve.SamuraiSwerveDrive;
import lib.hardware.swerve.SwerveModuleBase;
import lib.hardware.swerve.requests.DriveFieldOriented;
import lib.hardware.swerve.requests.SamuraiSwerveRequest;
import lib.hardware.swerve.test.SamSwerveModule;

public class SamuraiSwerveDrivetrain extends SubsystemBase {
    public static final int odometryFrequencyHz = 250;
    public static final double wheelBase = Units.inchesToMeters(27.5); //Meters
    public static final double trackWidth = Units.inchesToMeters(19.5); //Meters
    
    private final Pigeon2 m_gyro = new Pigeon2(9);
    private final StatusSignal<Angle> yaw = m_gyro.getYaw();
    //public static final double wheelRadius = 0.0508; //Meters
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };
    
    private final SamuraiSwerveDrive drive = new SamuraiSwerveDrive(
        moduleTranslations,
        new SwerveModuleBase[] {
            new SamSwerveModule(0),
            new SamSwerveModule(1),
            new SamSwerveModule(2),
            new SamSwerveModule(3)
        },
        () -> new Rotation2d(yaw.getValue())
    );
    
    public Command getTeleopDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        Supplier<SamuraiSwerveRequest> reqgen = () -> new DriveFieldOriented().withSpeeds(new ChassisSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            omegaSupplier.getAsDouble()
        ));
        return run(() -> drive.acceptRequest(reqgen.get()));
    }

}
