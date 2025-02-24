package lib.hardware.swerve;


import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.atomic.AtomicReferenceArray;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public class SamuraiSwerveDrive implements SwerveDriveBase {

    private SwerveModuleBase[] modules;

    private final Lock moduleLock = new ReentrantLock();
    private final Supplier<Rotation2d> headingSupplier; //TODO: Make SwerveIMU interface
    private final SwerveDriveKinematics kinematics;
    
    //TODO: redo the constructor to use the config objects
    public SamuraiSwerveDrive(Translation2d[] moduleTranslations,Supplier<Rotation2d> headingSupplier) {
        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
        this.headingSupplier = headingSupplier;
    }

    public SwerveModuleBase[] getModules() {
        return modules;
    }

    /** Implements high-speed odometry and pose estimation for the swerve drive */
    private class Swervometer {
        
        private ConcurrentSwerveState state = new ConcurrentSwerveState();

        private final Lock signalLock = new ReentrantLock();
        private final Lock estimatorLock = new ReentrantLock();

        private final SwerveDrivePoseEstimator poseEstimator;

        private SwerveState cachedState; //Odometry thread local

        private SwerveModulePosition[] oldPositions = new SwerveModulePosition[4];
        private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        private volatile Optional<Consumer<SwerveState>> telemetryConsumer = Optional.empty(); //TODO: See if this needs to be protected with a lock

        private Notifier notifier = new Notifier(this::run);

        private final ArrayList<BooleanSupplier> errorSignals = new ArrayList<>();

        public Swervometer(){
            poseEstimator = new SwerveDrivePoseEstimator(kinematics, headingSupplier.get(), oldPositions, new Pose2d(0,0,headingSupplier.get())); //TODO: Instantiate thiw with proper parameters
            notifier.setName("Swervometer");
        }

        public void start(){
            notifier.startPeriodic(1.0/250.0); //TODO: Make this configurable
        }

        public void registerTelemetryConsumer(Consumer<SwerveState> telemetryConsumer) {
            this.telemetryConsumer = Optional.of(telemetryConsumer);
        }

        /**Registers an error signal from the main thread. If any error signals are detected, then the thread does not register odometry for any devices during the given odometry cycle */
        public void registerErrorSignal(BooleanSupplier errorSignal){
            synchronized (signalLock) {
                errorSignals.add(errorSignal);
            }
        }

        public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3,N1> stdDevs) {
            synchronized (estimatorLock) {
                poseEstimator.addVisionMeasurement(pose,timestampSeconds,stdDevs);
            }
        }


        /**Periodic function to run in the odometry thread, updates pose estimator with latest odometry values*/
        private void run(){

            double timestamp = RobotController.getFPGATime()/1000000.0;
            //cachedState = state.get();
            Pose2d pose;

            synchronized (signalLock){
                for (int i = 0; i < errorSignals.size(); i++){
                    if (errorSignals.get(i).getAsBoolean()){
                        state.logFailedDaq();
                        return;
                    }
                }
            }
            
            SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            // oldPositions = cachedState.modulePositions;
            synchronized (moduleLock) {
                for (int i = 0; i < 4; i++) {
                    moduleStates[i] = modules[i].getState();
                    modulePositions[i] = modules[i].getPosition();
                }
            }
            /* 
            for (int i = 0; i < 4; i++) {
                moduleDeltas[i] = new SwerveModulePosition(
                    modulePositions[i].distanceMeters - oldPositions[i].distanceMeters,
                    modulePositions[i].angle.minus(oldPositions[i].angle)
                );
            }
            */
            synchronized (estimatorLock) {
                pose = poseEstimator.updateWithTime(timestamp,headingSupplier.get(),modulePositions);
            }

            state.logSuccessfulDaq(
                timestamp,
                modulePositions,
                moduleStates,
                pose,
                kinematics.toChassisSpeeds(moduleStates),
                headingSupplier.get()
            );
            
    
        }
    }

}
