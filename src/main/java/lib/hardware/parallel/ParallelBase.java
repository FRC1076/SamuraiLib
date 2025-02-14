package lib.hardware.parallel;

/** NOTE: ONLY HANDLES THE DATA CHANNELS. CONTROL SIGNALS MUST BE IMPLEMENTED EXTERNALLY */
public interface ParallelBase extends AutoCloseable {
    abstract int[] getChannels();
    abstract int[] getHALPortHandles();
    abstract int getWidth();
}
