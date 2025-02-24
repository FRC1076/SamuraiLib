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
import lib.hardware.swerve.requests.SamuraiSwerveRequest;

public class SamuraiSwerveDrive implements SwerveDriveBase {

    private SwerveModuleBase[] modules;

    private final Swervometer swervometer;

    private final Lock moduleLock = new ReentrantLock();
    private final Supplier<Rotation2d> headingSupplier; //TODO: Make SwerveIMU interface
    private final SwerveDriveKinematics kinematics;
    private final SwerveOptimizer optimizer;
    
    //TODO: make swerve drive factory that uses the configurator objects
    public SamuraiSwerveDrive(Translation2d[] moduleTranslations,SwerveModuleBase[] modules, Supplier<Rotation2d> headingSupplier) {
        this.modules = modules;
        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
        this.optimizer = new SwerveOptimizer(kinematics, 5, 5, 5, new Translation2d()); //TODO: UPDATE MAX SPEEDS
        this.headingSupplier = headingSupplier;
        swervometer = new Swervometer();
        swervometer.start();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void tareEverything() {
        // TODO Auto-generated method stub
    }

    @Override
    public void resetPose(Pose2d pose) {
        swervometer.resetPoseEstimator(pose);
    }

    @Override
    public SwerveState getState() {
        return swervometer.getState();
    }

    @Override
    public void registerTelemetry(Consumer<SwerveState> telemetryConsumer) {
        swervometer.registerTelemetryConsumer(telemetryConsumer);
    }

    @Override
    public void acceptRequest(SamuraiSwerveRequest request) {
        var desiredStates = request.getModuleStates(optimizer,getState());
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        swervometer.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public SwerveModuleBase[] getModules() {
        return modules;
    }

    /** Implements multithreaded high-speed odometry and pose estimation for the swerve drive */
    private class Swervometer {

        private static class ConcurrentSwerveState {

            private Lock stateLock = new ReentrantLock();
            
            private volatile int successfulDaqs;
            private volatile int failedDaqs;
            private volatile Pose2d pose;
        
            private volatile SwerveModulePosition[] modulePositions;
            private volatile SwerveModuleState[] moduleStates;
            private volatile SwerveModuleState[] moduleTargets;
        
            private volatile double timestamp;
            private volatile ChassisSpeeds speeds;
            private volatile Rotation2d rawHeading;
            private volatile double odometryPeriod;
        
            public SwerveState get() {
                var state = new SwerveState();
                
                synchronized (stateLock) {
                    mapToState(state);
                }
        
                return state;
            }

            private void mapToState(SwerveState state) {
                state.timestamp = timestamp;
                state.successfulDaqs = successfulDaqs;
                state.failedDaqs = failedDaqs;
                state.modulePositions = modulePositions.clone();
                state.moduleStates = moduleStates.clone();
                state.moduleTargets = moduleTargets.clone();
                state.pose = pose;
                state.speeds = speeds;
                state.rawHeading = rawHeading;
                state.odometryPeriod = odometryPeriod;
            }
        
            public SwerveState logSuccessfulDaq(
                double timestamp,
                SwerveModulePosition[] modulePositions,
                SwerveModuleState[] moduleStates,
                Pose2d pose,
                ChassisSpeeds speeds,
                Rotation2d rawHeading
            ) {
                SwerveState state = new SwerveState();
                synchronized (stateLock) {
                    this.timestamp = timestamp;
                    this.modulePositions = modulePositions;
                    this.moduleStates = moduleStates;
                    this.pose = pose;
                    this.speeds = speeds;
                    this.rawHeading = rawHeading;
                    this.successfulDaqs++;
                    mapToState(state);
                }
                return state;
            }
        
            public SwerveState logFailedDaq() {
                SwerveState state = new SwerveState();
                synchronized (stateLock) {
                    this.failedDaqs++;
                    mapToState(state);
                }
                return state;
            }
        
            public void resetDaqs() {
                synchronized (stateLock) {
                    this.successfulDaqs = 0;
                    this.failedDaqs = 0;
                }
            }
        
        }
        
        private ConcurrentSwerveState volatileState = new ConcurrentSwerveState();

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
            for (SwerveModuleBase module : modules) {
                registerErrorSignal(module.getErrorSignal());
            }
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
                        volatileState.logFailedDaq();
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

            synchronized (estimatorLock) {
                pose = poseEstimator.updateWithTime(timestamp,headingSupplier.get(),modulePositions);
            }

            var state = volatileState.logSuccessfulDaq(
                timestamp,
                modulePositions,
                moduleStates,
                pose,
                kinematics.toChassisSpeeds(moduleStates),
                headingSupplier.get()
            );

            telemetryConsumer.ifPresent(
                (telemetry) -> telemetry.accept(state)
            );
            
    
        }

        public SwerveState getState() {
            return volatileState.get();
        }

        public void resetPoseEstimator(Pose2d pose) {
            synchronized (estimatorLock) {    
                poseEstimator.resetPose(pose);
            }
        }
    }

}
