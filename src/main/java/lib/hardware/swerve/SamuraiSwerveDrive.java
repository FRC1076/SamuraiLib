package lib.hardware.swerve;


import java.util.ArrayList;
import java.util.Collection;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        
        private volatile SwerveState state;

        private final Lock stateLock = new ReentrantLock();
        private final Lock signalLock = new ReentrantLock();
        private final Lock estimatorLock = new ReentrantLock();

        private final SwerveDrivePoseEstimator poseEstimator;

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


        /**Periodic function to run in the odometry thread, updates pose estimator with latest odometry values*/
        private void run(){

            long timestamp = RobotController.getFPGATime();

            synchronized (signalLock){
                for (int i = 0; i < errorSignals.size(); i++){
                    if (errorSignals.get(i).getAsBoolean()){
                        synchronized (stateLock) {
                            state.failedDaqs++;
                        }
                        return;
                    }
                }
            }
            
            SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            oldPositions = state.modulePositions; //TODO: Check if this call needs to be synchronized on stateLock
            synchronized (moduleLock) {
                for (int i = 0; i < 4; i++) {
                    moduleStates[i] = modules[i].getState();
                    modulePositions[i] = modules[i].getPosition();
                }
            }
            for (int i = 0; i < 4; i++) {
                moduleDeltas[i] = new SwerveModulePosition(
                    modulePositions[i].distanceMeters - oldPositions[i].distanceMeters,
                    modulePositions[i].angle.minus(oldPositions[i].angle)
                );
            }
            synchronized (estimatorLock) {
                poseEstimator.updateWithTime(timestamp/1000000.0,headingSupplier.get(),modulePositions);
            }
            synchronized (stateLock) {
                state.successfulDaqs++;
                state.modulePositions = modulePositions;
                state.moduleStates = moduleStates;
                state.timestamp = timestamp;
                state.rawHeading = headingSupplier.get();
                state.pose = poseEstimator.getEstimatedPosition();
                if (telemetryConsumer.isPresent()) {
                    telemetryConsumer.get().accept(state.clone());
                }
            }
    
        }
    }

}
