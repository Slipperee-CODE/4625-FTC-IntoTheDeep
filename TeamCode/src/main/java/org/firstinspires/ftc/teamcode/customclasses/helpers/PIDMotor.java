package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDMotor {
    private final Clock clock = new Clock();
    private DcMotor motor;
    protected double p, i, d;
    protected double errorSum;
    protected int lastError = Integer.MAX_VALUE;
    protected int target;
    public static final double POWER_THRESHOLD = 0.1; // when calculated power is below this number, it will round to 0 so motor doesn't become super hot
    protected static final int INTEGRAL_START_THRESHOLD = 20; // how many encoder ticks the delta error must be below to activate the error sum

    public PIDMotor(DcMotor motor,double p, double i, double d)
    {
        this.motor = motor;
        if (motor != null) {
            motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        }
        this.p = p; this.i = i; this.d = d;
    }

    protected double clamp(double x, double min, double max) {
        return Math.min(Math.max(x,min),max);
    }

    protected double round(double x) {
        return Math.round(x*1000)/1000.0;
    }

    public void setRawPower(double power) {
        this.motor.setPower(power);
    }

    public void setTarget(int target) {
        if (target != this.target) {
            this.target = target; this.errorSum = 0;
        }
    }

    public int getTarget() {return target;}

    public int getPos() {return  motor.getCurrentPosition();}

    public void ResetPID()
    {
        // Will reset
        target = 0;
        errorSum = 0;

        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    public double[] getPID() {
        return new double[]{p,i,d};
    }

    public void setPID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public int getError()
    {
        final int error = target - motor.getCurrentPosition();
        if (Integer.MAX_VALUE == lastError) {
            lastError = error;
        }
        return error;
    }

    public void Update() { Update(null,clock.getDeltaSeconds()); }

    public void Update(double deltaTime) { Update(null,deltaTime);}

    public void Update(Telemetry telemetry) { Update(telemetry,clock.getDeltaSeconds());}

    public void Update(Telemetry telemetry, double deltaTime)
    {
        final double pOutput;
        final double iOutput;
        final double dOutput;

        int error = getError();
        if (error * lastError < 0) {
            // error crossed signs
            errorSum = 0.0;
        }
        pOutput = p * error;

        //Must be negative to "slow" down the effects of a large spike
        dOutput = -d * (error - lastError) / deltaTime;
        if (error - lastError < INTEGRAL_START_THRESHOLD) {
            errorSum += error * deltaTime * i;
        }

        errorSum = clamp(errorSum,-1.0,1.0);
        iOutput = errorSum;

        final double output = pOutput + iOutput + dOutput;
        if (Math.abs(output) > POWER_THRESHOLD)
            motor.setPower(Math.tanh(output));
        else
            motor.setPower(0);
        if (telemetry != null) {
            telemetry.addLine("Output -> P: " + round(pOutput) + "  I: " + round(iOutput) + " D: " + round(dOutput));
        }
    }
}