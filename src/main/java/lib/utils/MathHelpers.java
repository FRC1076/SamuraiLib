package lib.utils;

public final class MathHelpers {

    /**
     * Clamps a value between a minimum and maximum value.
     * @param value The value to clamp.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return The clamped value.
     */
    public static double clamp(double value, double min, double max) {
        if(min > max) {
            throw new IllegalArgumentException("min must be less than max");
        }

        return Math.max(min, Math.min(max, value));
    }
}
