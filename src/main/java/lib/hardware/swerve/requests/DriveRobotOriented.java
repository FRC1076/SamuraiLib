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
import lib.hardware.swerve.SwerveDrive.ControlParameters;
import lib.hardware.swerve.SwerveDrive.SwerveState;

public class DriveRobotOriented implements SamuraiSwerveRequest {
    
    private double vxRobot = 0;
    private double vyRobot = 0;
    private double omega = 0;
    private double[] xForceFeedforwards = {0,0,0,0};
    private double[] yForceFeedforwards = {0,0,0,0};
    private boolean desaturateWheelSpeeds = true;
    private Translation2d centerOfRotation = new Translation2d();

    public DriveRobotOriented withCenterOfRotation(Translation2d centerOfRotation) {
        this.centerOfRotation = centerOfRotation;
        return this;
    }

    public DriveRobotOriented withDesaturateWheelSpeeds(boolean enabled) {
        this.desaturateWheelSpeeds = enabled;
        return this;
    }

    public DriveRobotOriented withSpeeds(ChassisSpeeds speeds) {
        this.vxRobot = speeds.vxMetersPerSecond;
        this.vyRobot = speeds.vyMetersPerSecond;
        this.omega = speeds.omegaRadiansPerSecond;
        return this;
    }

    public DriveRobotOriented withVx(double vxMetersPerSecond) {
        this.vxRobot = vxMetersPerSecond;
        return this;
    }

    public DriveRobotOriented withVy(double vyMetersPerSecond) {
        this.vyRobot = vyMetersPerSecond;
        return this;
    }

    public DriveRobotOriented withOmega(double omegaRadiansPerSecond) {
        this.omega = omegaRadiansPerSecond;
        return this;
    }

    public DriveRobotOriented withVx(LinearVelocity vx) {
        this.vxRobot = vx.in(MetersPerSecond);
        return this;
    }

    public DriveRobotOriented withVy(LinearVelocity vy) {
        this.vyRobot = vy.in(MetersPerSecond);
        return this;
    }

    public DriveRobotOriented withOmega(AngularVelocity omega) {
        this.omega = omega.in(RadiansPerSecond);
        return this;
    }

    public DriveRobotOriented withXForceFeedforwardsNewtons(double[] xForceFeedforwards) {
        this.xForceFeedforwards = xForceFeedforwards;
        return this;
    }

    public DriveRobotOriented withYForceFeedforwardsNewtons(double[] yForceFeedforwards) {
        this.yForceFeedforwards = yForceFeedforwards;
        return this;
    }

    public DriveRobotOriented withXForceFeedforwards(Force[] xForceFeedforwards) {

        for (int i = 0; i < 4; i++) {
            this.xForceFeedforwards[i] = xForceFeedforwards[i].in(Newtons);
        }

        return this;
    }

    public DriveRobotOriented withYForceFeedforwards(Force[] yForceFeedforwards) {
        
        for (int i = 0; i < 4; i++) {
            this.yForceFeedforwards[i] = yForceFeedforwards[i].in(Newtons);
        }
        
        return this;
    }

    @Override
    public SwerveModuleState[] getModuleStates(SwerveOptimizer optimizer, ControlParameters cparams, SwerveState state) {
        var robotSpeeds = new ChassisSpeeds(vxRobot,vyRobot,omega);
        var moduleStates = desaturateWheelSpeeds
            ? optimizer.calculateModuleStates(robotSpeeds,state.moduleStates)
            : optimizer.calculateModuleStatesNoDesaturation(robotSpeeds,state.moduleStates);
        return moduleStates;
    }

    @Override
    public double[] getArbParams() {
        return new double[] {
            vxRobot,
            vyRobot,
            omega,
            centerOfRotation.getX(),
            centerOfRotation.getY(),
            desaturateWheelSpeeds ? 1.0 : 0.0
        };
    }

    @Override
    public SwerveRequestType getRequestType() {
        return SwerveRequestType.kDriveRobotOriented;
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
