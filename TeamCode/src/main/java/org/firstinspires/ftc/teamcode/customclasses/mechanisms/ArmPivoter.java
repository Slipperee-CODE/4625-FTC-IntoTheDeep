package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;
import org.firstinspires.ftc.teamcode.customclasses.helpers.PIDMotor;

public class ArmPivoter extends Mechanism {
    public enum PivotPos{
        DEFAULT_PIVOT(0),
        LOWER_HANG_PIVOT(convertPercentAngleToTicks(0.33)),
        UPPER_BUCKET_PIVOT(convertPercentAngleToTicks(.8)),
        LOWER_BUCKET_PIVOT(convertPercentAngleToTicks(.8)),
        UPPER_SPECIMEN_BAR_PIVOT(convertPercentAngleToTicks(0.66)),
        LOWER_SPECIMEN_BAR_PIVOT(convertPercentAngleToTicks(0.33));

        int pos;
        PivotPos(int pos) {this.pos = pos;}
    }

    private PIDMotor leftPivotPIDMotor = null;
    private PIDMotor rightPivotPIDMotor = null;

    public static final double P = 0.0045;
    public static final double I = 0.00001;
    public static final double D = 0.00;

    private static final double TICKS_PER_REV = 384.5;
    private static final int SMALL_PULLEY_TEETH = 18;
    private static final int BIG_PULLEY_TEETH = 122;
    private static final double PULLEY_RATIO = (double) BIG_PULLEY_TEETH / SMALL_PULLEY_TEETH;
    private static final double TICKS_FOR_90_DEGREES = TICKS_PER_REV * 0.25 * PULLEY_RATIO;
    private final int TOLERANCE = 10; // in ticks

    public ArmPivoter(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public ArmPivoter(HardwareMap hardwareMap){
        DcMotor leftPivotMotor = hardwareMap.get(DcMotor.class, "leftPivotMotor");
        DcMotor rightPivotMotor = hardwareMap.get(DcMotor.class, "rightPivotMotor");

        leftPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivotPIDMotor = new PIDMotor(leftPivotMotor, P, I, D);
        rightPivotPIDMotor = new PIDMotor(rightPivotMotor, P, I, D);
        leftPivotPIDMotor.ResetPID();
        rightPivotPIDMotor.ResetPID();
    }

    @Override
    public void update(Telemetry telemetry) {
        leftPivotPIDMotor.update();
        rightPivotPIDMotor.update();
        telemetry.addData("leftPivotPIDMotor Pos:", leftPivotPIDMotor.getPos());
        telemetry.addData("leftPivotPIDMotor Target:", leftPivotPIDMotor.getTarget());
        telemetry.addData("rightPivotPIDMotor Pos:", rightPivotPIDMotor.getPos());
        telemetry.addData("rightPivotPIDMotor Target:", rightPivotPIDMotor.getTarget());
    }

    @Override
    public void update() {

    }

    private static int convertPercentAngleToTicks(double percentOf90){
        return (int) (TICKS_FOR_90_DEGREES*percentOf90);
    }

    public void SetPivot(PivotPos pivotPos){
        leftPivotPIDMotor.setTarget(pivotPos.pos);
        rightPivotPIDMotor.setTarget(pivotPos.pos);
    }
    
    public double GetCurrPivotInRadians(){
        return (((leftPivotPIDMotor.getPos() + rightPivotPIDMotor.getPos()) / 2.0f)/TICKS_FOR_90_DEGREES)*Math.PI/2;
    }

    public boolean isPIDMotorTargetsReached(){
        return Math.abs(leftPivotPIDMotor.getError()) < TOLERANCE && Math.abs(rightPivotPIDMotor.getError()) < TOLERANCE;
    }
}
