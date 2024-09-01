package org.firstinspires.ftc.teamcode.customclasses.helpers;

public class Clock {
    private static final double NANOSECONDS_PER_SECOND = 1_000_000_000;
    private static final double MILLISECONDS_PER_SECOND = 1_000;
    private static final long NANOSECONDS_PER_MILLISECOND = (long) (NANOSECONDS_PER_SECOND / MILLISECONDS_PER_SECOND);
    private long startTime;
    public Clock() {
        startTime = getNs();
    }
    public long getNs() {
        return System.nanoTime();
    }

    public double getTimeSeconds() {
        return getTimeNano() /  NANOSECONDS_PER_SECOND;
    }
    public long getTimeNano() {
        return getNs() - startTime;
    }

    // This may or may not be a side effect, which affects the purity of this method.
    // This resets <startTime> to the current time which may be undesired in some situations
    public long tick() {
        final long currTime = getNs();
        final long time = currTime - startTime;
        startTime = currTime;
        return time;
    }
    public void reset() {
        startTime = getNs();
    }
    public double getDeltaSeconds() {
        return tick() / NANOSECONDS_PER_SECOND;
    }

    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    public static void sleepPrecise(long milliseconds) {
        final long waitTime = milliseconds * NANOSECONDS_PER_MILLISECOND;
        final long INNER_SLEEP_TIME = 10L;
        long startNano = System.nanoTime();
        while (System.nanoTime() - startNano < waitTime) {
            sleep(INNER_SLEEP_TIME);
        }
    }
}
