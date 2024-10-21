package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import static org.firstinspires.ftc.teamcode.customclasses.mechanisms.ArmExtender.ExtensionPos.MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.customclasses.mechanisms.ArmExtender.ExtensionPos.MIN_EXTENSION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;
import org.firstinspires.ftc.teamcode.customclasses.helpers.PIDMotor;

public class ArmExtender extends Mechanism {
    public enum ExtensionPos{
        MIN_EXTENSION(0),
        MAX_EXTENSION(0),
        DEFAULT_EXTENSION(0),
        SUBMERSIBLE_EXTENSION(0),
        LOWER_BUCKET_EXTENSION(0),
        UPPER_BUCKET_EXTENSION(0),
        LOWER_SPECIMEN_BAR_EXTENSION(0),
        UPPER_SPECIMEN_BAR_EXTENSION(0),
        LOWER_HANG_EXTENSION(0);

        int pos;
        ExtensionPos(int pos) {this.pos = pos;}
    }

    private PIDMotor farPivotPIDMotor = null;
    private PIDMotor closePivotPIDMotor = null;

    public static final double P = 0.0045;
    public static final double I = 0.00001;
    public static final double D = 0.00;

    private static final float SPEED = 50.0f;

    public ArmExtender(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public ArmExtender(HardwareMap hardwareMap){
        DcMotor farPivotMotor = hardwareMap.get(DcMotor.class, "farPivotMotor");
        DcMotor closePivotMotor = hardwareMap.get(DcMotor.class, "closePivotMotor");

        farPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closePivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        farPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        closePivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        farPivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        farPivotPIDMotor = new PIDMotor(farPivotMotor, P, I, D);
        closePivotPIDMotor = new PIDMotor(closePivotMotor, P, I, D);
        farPivotPIDMotor.ResetPID();
        closePivotPIDMotor.ResetPID();
    }

    public ArmExtender(CustomGamepad gamepad){
        this.gamepad = gamepad;
    }

    @Override
    public void update() {
        if (gamepad != null) {
            float right_stick_y = -gamepad.right_stick_y;
            if (right_stick_y != 0) {
                int targetLeft = farPivotPIDMotor.getTarget() + (int) (right_stick_y * SPEED);
                int targetRight = closePivotPIDMotor.getTarget() + (int) (right_stick_y * SPEED);
                int clippedRight = Range.clip(targetRight, MIN_EXTENSION.pos, MAX_EXTENSION.pos);
                int clippedLeft = Range.clip(targetLeft, MIN_EXTENSION.pos, MAX_EXTENSION.pos);

                farPivotPIDMotor.setTarget(clippedLeft);
                closePivotPIDMotor.setTarget(clippedRight);
            }
        }

        farPivotPIDMotor.update();
        closePivotPIDMotor.update();
    }

    @Override
    public void update(Telemetry telemetry) {
        update();
    }

    public void SetExtension(ExtensionPos extensionPos){
        farPivotPIDMotor.setTarget(extensionPos.pos);
        closePivotPIDMotor.setTarget(extensionPos.pos);
    }
}
