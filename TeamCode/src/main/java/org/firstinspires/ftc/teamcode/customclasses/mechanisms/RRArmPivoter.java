package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;
import org.firstinspires.ftc.teamcode.customclasses.helpers.PIDMotor;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRMechanism;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRPIDMotor;

import java.util.List;

public class RRArmPivoter extends RRMechanism {
    public enum PivotPos{
        DEFAULT_PIVOT(0),
        SAFE_DEFAULT_PIVOT(convertPercentAngleToTicks(0)),
        LOWER_HANG_PIVOT(convertPercentAngleToTicks(0.33)),
        UPPER_BUCKET_PIVOT(convertPercentAngleToTicks(1.05)),
        UPPER_BUCKET_PIVOT_BUT_MORE_FORWARD(convertPercentAngleToTicks(.85)),
        LOWER_BUCKET_PIVOT(convertPercentAngleToTicks(.95)),
        UPPER_SPECIMEN_BAR_PIVOT(convertPercentAngleToTicks(0.6)),
        LOWER_SPECIMEN_BAR_PIVOT(convertPercentAngleToTicks(0.33)),
        WALL_GRAB_PIVOT(convertPercentAngleToTicks(0.185));

        int pos;
        PivotPos(int pos) {this.pos = pos;}
    }

    public RRPIDMotor leftPivotPIDMotor = null;
    public RRPIDMotor rightPivotPIDMotor = null;

    public static final double P = 0.003; //lower this to 0.0035
    public static final double I = 0.00001;
    public static final double D = 0.000015;

    private static final double TICKS_PER_REV = 384.5;
    private static final int SMALL_PULLEY_TEETH = 18;
    private static final int BIG_PULLEY_TEETH = 122;
    private static final double PULLEY_RATIO = (double) BIG_PULLEY_TEETH / SMALL_PULLEY_TEETH;
    private static final double TICKS_FOR_90_DEGREES = TICKS_PER_REV * 0.25 * PULLEY_RATIO;
    private final int TOLERANCE = 10; // in ticks

    public RRArmPivoter(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public RRArmPivoter(HardwareMap hardwareMap){
        DcMotor leftPivotMotor = hardwareMap.get(DcMotor.class, "leftPivotMotor");
        DcMotor rightPivotMotor = hardwareMap.get(DcMotor.class, "rightPivotMotor");

        leftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivotPIDMotor = new RRPIDMotor(leftPivotMotor, P, I, D);
        rightPivotPIDMotor = new RRPIDMotor(rightPivotMotor, P, I, D);
        leftPivotPIDMotor.ResetPID();
        rightPivotPIDMotor.ResetPID();
    }

    private static int convertPercentAngleToTicks(double percentOf90){
        return (int) (TICKS_FOR_90_DEGREES*percentOf90);
    }

    public void SetPivot(PivotPos pivotPos){
        if (pivotPos==PivotPos.DEFAULT_PIVOT){
            leftPivotPIDMotor.ResetPID();
            rightPivotPIDMotor.ResetPID();
            return;
        }
        leftPivotPIDMotor.setTarget(pivotPos.pos);
        rightPivotPIDMotor.setTarget(pivotPos.pos);
    }
    
    public double GetCurrPivotInRadians(){
        return (((leftPivotPIDMotor.getPos() + rightPivotPIDMotor.getPos()) / 2.0f)/TICKS_FOR_90_DEGREES)*Math.PI/2;
    }

    @Override
    public void queueActions() {

    }

    @Override
    public Action queueUpdateActions() {
        return new ParallelAction(
                leftPivotPIDMotor.updateAction(),
                rightPivotPIDMotor.updateAction()
        );
    }

    public void deactivate(){
        leftPivotPIDMotor.deactivate();
        rightPivotPIDMotor.deactivate();
    }
}
