package lib.hardware.swerve.requests;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import lib.hardware.swerve.SwerveOptimizer;
import lib.hardware.swerve.SwerveDriveBase.ControlParameters;
import lib.hardware.swerve.SwerveDriveBase.SwerveState;

// All native units are in SI base units unless otherwise specified
public class DriveFieldOriented implements SamuraiSwerveRequest {
    private double vxField = 0;
    private double vyField = 0;
    private double omega = 0;
    private double[] xForceFeedforwards = {0,0,0,0};
    private double[] yForceFeedforwards = {0,0,0,0};
    private boolean desaturateWheelSpeeds = true;
    private Translation2d centerOfRotation = new Translation2d();

    public DriveFieldOriented withCenterOfRotation(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
        return this;
    }

    public DriveFieldOriented withSpeeds(ChassisSpeeds speeds) {
        this.vxField = speeds.vxMetersPerSecond;
        this.vyField = speeds.vyMetersPerSecond;
        this.omega = speeds.omegaRadiansPerSecond;
        return this;
    }

    public DriveFieldOriented withVx(double vxMetersPerSecond) {
        this.vxField = vxMetersPerSecond;
        return this;
    }

    public DriveFieldOriented withVy(double vyMetersPerSecond) {
        this.vyField = vyMetersPerSecond;
        return this;
    }

    public DriveFieldOriented withOmega(double omegaRadiansPerSecond) {
        this.omega = omegaRadiansPerSecond;
        return this;
    }

    public DriveFieldOriented withVx(LinearVelocity vx) {
        this.vxField = vx.in(MetersPerSecond);
        return this;
    }

    public DriveFieldOriented withVy(LinearVelocity vy) {
        this.vyField = vy.in(MetersPerSecond);
        return this;
    }

    public DriveFieldOriented withOmega(AngularVelocity omega) {
        this.omega = omega.in(RadiansPerSecond);
        return this;
    }

    public DriveFieldOriented withXForceFeedforwardsNewtons(double[] xForceFeedforwards) {
        this.xForceFeedforwards = xForceFeedforwards;
        return this;
    }

    public DriveFieldOriented withYForceFeedforwardsNewtons(double[] yForceFeedforwards) {
        this.yForceFeedforwards = yForceFeedforwards;
        return this;
    }

    public DriveFieldOriented withXForceFeedforwards(Force[] xForceFeedforwards) {

        for (int i = 0; i < 4; i++) {
            this.xForceFeedforwards[i] = xForceFeedforwards[i].in(Newtons);
        }

        return this;
    }

    public DriveFieldOriented withYForceFeedforwards(Force[] yForceFeedforwards) {
        
        for (int i = 0; i < 4; i++) {
            this.yForceFeedforwards[i] = yForceFeedforwards[i].in(Newtons);
        }

        return this;
    }


    public DriveFieldOriented withDesaturateWheelSpeeds(boolean enabled) {
        this.desaturateWheelSpeeds = enabled;
        return this;
    }

    @Override
    public SwerveModuleState[] getModuleStates(SwerveOptimizer optimizer, ControlParameters cparams, SwerveState state) {
        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxField,vyField,omega,state.pose.getRotation());
        var moduleStates = desaturateWheelSpeeds
            ? optimizer.calculateModuleStates(robotSpeeds)
            : optimizer.calculateModuleStatesNoDesaturation(robotSpeeds);
        return moduleStates;
    }

    @Override
    public double[] getArbParams() {
        return new double[] {
            vxField,
            vyField,
            omega,
            centerOfRotation.getX(),
            centerOfRotation.getY(),
            desaturateWheelSpeeds ? 1.0 : 0.0
        };
    }

    @Override
    public SwerveRequestType getRequestType() {
        return SwerveRequestType.kDriveFieldOriented;
    }
    
    @Override
    public double[] getXForceFeedforwardsNewtons() {
        return xForceFeedforwards;
    }

    @Override
    public double[] getYForceFeedforwardsNewtons() {
        return yForceFeedforwards;
    }
}
