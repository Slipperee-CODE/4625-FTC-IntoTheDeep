package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import static org.firstinspires.ftc.teamcode.customclasses.mechanisms.ArmExtender.ExtensionPos.MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.customclasses.mechanisms.ArmExtender.ExtensionPos.MIN_EXTENSION;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;
import org.firstinspires.ftc.teamcode.customclasses.helpers.PIDMotor;

public class ArmExtender extends Mechanism {
    public enum ExtensionPos{
        MIN_EXTENSION(0),
        MAX_EXTENSION(convertInchesToTicks(20)),
        DEFAULT_EXTENSION(convertInchesToTicks(0)),
        SUBMERSIBLE_EXTENSION(convertInchesToTicks(15)),
        LOWER_BUCKET_EXTENSION(convertInchesToTicks(10)),
        UPPER_BUCKET_EXTENSION(convertInchesToTicks(12)),
        LOWER_SPECIMEN_BAR_EXTENSION(convertInchesToTicks(10)),
        UPPER_SPECIMEN_BAR_EXTENSION(convertInchesToTicks(12)),
        LOWER_HANG_EXTENSION(convertInchesToTicks(15));

        int pos;
        ExtensionPos(int pos) {this.pos = pos;}
    }

    private PIDMotor farPivotPIDMotor = null;
    private PIDMotor closePivotPIDMotor = null;
    private DigitalChannel magneticLimitSwitch;

    public static final double P = 0.0045;
    public static final double I = 0.00001;
    public static final double D = 0.00;

    private static final float SPEED = 50.0f;
    private static final double TICKS_PER_REV = 751.8;
    private static final double SPOOL_CIRCUMFERENCE = 2*0.575*Math.PI; //in inches

    public ArmExtender(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public ArmExtender(HardwareMap hardwareMap){
        DcMotor farPivotMotor = hardwareMap.get(DcMotor.class, "farPivotMotor");
        DcMotor closePivotMotor = hardwareMap.get(DcMotor.class, "closePivotMotor");
        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "magneticLimitSwitch");

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

        if (magneticLimitSwitch.getState()){
            farPivotPIDMotor.ResetPID();
            closePivotPIDMotor.ResetPID();
        }

        farPivotPIDMotor.update();
        closePivotPIDMotor.update();
    }

    @Override
    public void update(Telemetry telemetry) {
        update();
    }

    private static int convertInchesToTicks(double inches){
        return (int) ((inches/SPOOL_CIRCUMFERENCE) * TICKS_PER_REV);
    }

    public void SetExtension(ExtensionPos extensionPos){
        farPivotPIDMotor.setTarget(extensionPos.pos);
        closePivotPIDMotor.setTarget(extensionPos.pos);
    }
}
