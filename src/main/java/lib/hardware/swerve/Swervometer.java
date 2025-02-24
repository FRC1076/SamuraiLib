package lib.hardware.swerve;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import lib.hardware.swerve.SwerveDriveBase.SwerveState;

class Swervometer {
        
    //SHARED RESOURCES
    private final ArrayList<SwerveModuleBase> loggedModules = new ArrayList<>();

    private AtomicReference<SwerveState> atomicSwerveState;

    private final Lock stateLock = new ReentrantLock();
    private final Lock signalLock = new ReentrantLock();

    private static Swervometer instance = null;
    private Notifier notifier = new Notifier(this::run);

    private final ArrayList<BooleanSupplier> errorSignals = new ArrayList<>();

    private Swervometer(){
        notifier.setName("OdometryThread");
    }

    public static Swervometer getInstance() {
        if (instance == null){
            instance = new Swervometer();
        }
        return instance;
    }

    public void start(){
        notifier.startPeriodic(1.0/250.0); //TODO: Make this configurable
    }

    /**Registers a module from the main thread*/
    public void registerModule(SwerveModuleBase module){
        synchronized (signalLock) {
            loggedModules.add(module);
        }
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

        for (int i = 0; i < errorSignals.size(); i++){
            if (errorSignals.get(i).getAsBoolean()){
                return;
            }
        }

        synchronized (stateLock) {
            SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++)
        }


        samplesSinceLastPoll.incrementAndGet(); //All queues are GUARANTEED to have at least samplesSinceLastPoll elements in them, all correctly ordered

    }
}

