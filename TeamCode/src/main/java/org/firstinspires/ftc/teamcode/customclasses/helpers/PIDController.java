package org.firstinspires.ftc.teamcode.customclasses.helpers;

public class PIDController {
    private final Clock clock = new Clock();
    private double p, i, d;
    private double errorSum;
    private double lastError = Double.NaN;
    public final double LOWER_THRESHOLD; // when calculated power is below this number, it will round to 0 so motor doesn't become as hot as
    private final double INTEGRAL_START_THRESHOLD; // how many encoder ticks the delta error must be below to activate the error sum
    public static final double MIN_INTEGRAL_VALUE = -1.0;
    public static final double MAX_INTEGRAL_VALUE = 1.0;


    public PIDController(double p) {
        this(p,0,0,.1,20);
    }
    public PIDController(double p, double i , double d) {
        this(p,i,d,.1,20);
    }
    public PIDController(double p, double i, double d, double lowerThreshold) { this(p,i,d,lowerThreshold,20); }
    public PIDController(double p, double i, double d, double lowerThreshold, double intStartThreshold)
    {
        LOWER_THRESHOLD = lowerThreshold;
        INTEGRAL_START_THRESHOLD = intStartThreshold;
        this.p = p;
        this.i = i;
        this.d = d;
    }

    private double clamp(double x, double min, double max) {
        return Math.min(Math.max(x,min),max);
    }

    public void ResetPID()
    {
        errorSum = 0;
    }

    public double[] getPID() {
        return new double[]{p,i,d};
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double Update(double error) { return Update(error,clock.getTimeSeconds());}

    public double Update(double error,double deltaTime)
    {
        final double pOutput;
        final double iOutput;
        final double dOutput;
        if (Double.isNaN(lastError)) lastError = error;
        if (error * lastError <= 0) {
            // error crossed signs
            errorSum = 0.0;
        }
        pOutput = p * error;

        //Must be negative to "slow" down the effects of a large spike
        dOutput = d * (error - lastError) / deltaTime;
        if (Math.abs(error - lastError) < INTEGRAL_START_THRESHOLD) {
            errorSum += error * deltaTime * i;
        }

        errorSum = clamp(errorSum,-MIN_INTEGRAL_VALUE,MAX_INTEGRAL_VALUE);
        iOutput = errorSum;

        final double output = pOutput + iOutput + dOutput;
        lastError = error;
        clock.reset();
        if (Math.abs(output) > LOWER_THRESHOLD)
            return (Math.tanh(output));
        else
            return 0.0;

    }
}