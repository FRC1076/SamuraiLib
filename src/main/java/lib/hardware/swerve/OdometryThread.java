package lib.hardware.swerve;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

class OdometryThread {
        
    //SHARED RESOURCES
    private final ArrayList<SwerveModuleBase> loggedModules = new ArrayList<>();
    private final ArrayList<ConcurrentLinkedQueue<SwerveModuleState>> moduleStateBuffers = new ArrayList<>();

    private final Lock signalLock = new ReentrantLock();

    private static OdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    private final ArrayList<BooleanSupplier> errorSignals = new ArrayList<>();
    private final ArrayList<ConcurrentLinkedQueue<Long>> timestampQueues = new ArrayList<>();

    //ODOMETRY THREAD ONLY
    private AtomicInteger samplesSinceLastPoll = new AtomicInteger(0);

    //MAIN THREAD ONLY
    public int sampleCount = 0;

    private OdometryThread(){
        notifier.setName("OdometryThread");
    }

    public static OdometryThread getInstance() {
        if (instance == null){
            instance = new OdometryThread();
        }
        return instance;
    }

    public void start(){
        notifier.startPeriodic(1.0/250.0); //TODO: Make this configurable
    }

    /**Registers a module from the main thread*/
    public ConcurrentLinkedQueue<SwerveModuleState> registerModule(SwerveModuleBase module){
        ConcurrentLinkedQueue<SwerveModuleState> buffer = new ConcurrentLinkedQueue<>();
        synchronized (signalLock) {
            loggedModules.add(module);
            moduleStateBuffers.add(buffer);
        }
        return buffer;
    }

    /**Registers an error signal from the main thread. If any error signals are detected, then the thread does not register odometry for any devices during the given odometry cycle */
    public void registerErrorSignal(BooleanSupplier errorSignal){
        synchronized (signalLock) {
            errorSignals.add(errorSignal);
        }
    }

    /**Makes a timestamp queue. Timestamps are recorded in microseconds as a Long */
    public ConcurrentLinkedQueue<Long> makeTimestampQueue(){
        ConcurrentLinkedQueue<Long> queue = new ConcurrentLinkedQueue<>();
        synchronized (signalLock) {
            timestampQueues.add(queue);
        }
        return queue;
    }

    /**Periodic function to run in the odometry thread, updates queues with latest odometry values*/
    private void run(){

        long timestamp = RobotController.getFPGATime();

        for (int i = 0; i < errorSignals.size(); i++){
            if (errorSignals.get(i).getAsBoolean()){
                return;
            }
        }

        synchronized (signalLock) {
            for (int i = 0; i < loggedModules.size(); i++){
                moduleStateBuffers.get(i).offer(loggedModules.get(i).getState());
            }
            for (int i = 0; i < timestampQueues.size(); i++){
                timestampQueues.get(i).offer(timestamp);
            }
        }


        samplesSinceLastPoll.incrementAndGet(); //All queues are GUARANTEED to have at least samplesSinceLastPoll elements in them, all correctly ordered

    }

    /** This method should be called ONCE per main-cycle thread */
    public void poll() {
        sampleCount = samplesSinceLastPoll.getAndSet(0);
    }

    /** Safely reads N elements of type T from a queue into a collection. Will not block if queue is empty*/
    public static <T> void safeDrain(ConcurrentLinkedQueue<T> source, Collection<T> dest, int n) {
        for (int i = 0; i < n; i++){
            dest.add(source.poll());
        }
    }
}

