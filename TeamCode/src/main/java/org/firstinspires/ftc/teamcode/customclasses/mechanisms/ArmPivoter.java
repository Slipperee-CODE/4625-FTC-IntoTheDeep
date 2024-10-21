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
        LOWER_HANG_PIVOT(0),
        UPPER_BUCKET_PIVOT(0),
        LOWER_BUCKET_PIVOT(0),
        UPPER_SPECIMEN_BAR_PIVOT(0),
        LOWER_SPECIMEN_BAR_PIVOT(0);

        int pos;
        PivotPos(int pos) {this.pos = pos;}
    }

    private PIDMotor leftPivotPIDMotor = null;
    private PIDMotor rightPivotPIDMotor = null;

    public static final double P = 0.0045;
    public static final double I = 0.00001;
    public static final double D = 0.00;
    private static final double TOLERANCE = .01;

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

        leftPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivotPIDMotor = new PIDMotor(leftPivotMotor, P, I, D);
        rightPivotPIDMotor = new PIDMotor(rightPivotMotor, P, I, D);
        leftPivotPIDMotor.ResetPID();
        rightPivotPIDMotor.ResetPID();
    }

    @Override
    public void update() {
        leftPivotPIDMotor.update();
        rightPivotPIDMotor.update();
    }

    @Override
    public void update(Telemetry telemetry) {
        update();
    }

    public void SetPivot(PivotPos pivotPos){
        leftPivotPIDMotor.setTarget(pivotPos.pos);
        rightPivotPIDMotor.setTarget(pivotPos.pos);
    }
}
