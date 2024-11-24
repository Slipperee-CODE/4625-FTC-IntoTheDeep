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
        MIN_EXTENSION(-10),
        MAX_EXTENSION(4000), //4400 max
        DEFAULT_EXTENSION(0),
        SUBMERSIBLE_EXTENSION(250),
        LOWER_BUCKET_EXTENSION(125),
        UPPER_BUCKET_EXTENSION(175),
        LOWER_SPECIMEN_BAR_EXTENSION(300),
        UPPER_SPECIMEN_BAR_EXTENSION(350),
        LOWER_HANG_EXTENSION(400);

        int pos;
        ExtensionPos(int pos) {this.pos = pos;}
    }

    private PIDMotor farPivotPIDMotor = null;
    private PIDMotor closePivotPIDMotor = null;
    private DigitalChannel magneticLimitSwitch;

    public static final double P = 0.0045;
    public static final double I = 0.00001;
    public static final double D = 0.00;

    private static final float SPEED = 100.0f;

    private boolean limitSwitchWasActiveLastFrame = false;

    private int effectiveCurrentMaxExtension;
    private final int HORIZONTAL_EXPANSION_LIMIT = 3000; //in ticks

    public ArmExtender(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public ArmExtender(HardwareMap hardwareMap){
        DcMotor farPivotMotor = hardwareMap.get(DcMotor.class, "farPivotMotor");
        DcMotor closePivotMotor = hardwareMap.get(DcMotor.class, "closePivotMotor");
        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "magneticLimitSwitch");
        magneticLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        farPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        closePivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        farPivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        closePivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        farPivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        closePivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closePivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        farPivotPIDMotor = new PIDMotor(farPivotMotor, P, I, D);
        closePivotPIDMotor = new PIDMotor(closePivotMotor, P, I, D);
        farPivotPIDMotor.ResetPID();
        closePivotPIDMotor.ResetPID();
    }

    public ArmExtender(CustomGamepad gamepad){
        this.gamepad = gamepad;
    }

    public void update(Telemetry telemetry, double approxCurrPivotInRadians) {
        if (gamepad != null) {
            float right_stick_y = -gamepad.right_stick_y;
            effectiveCurrentMaxExtension = Math.max((int)
                    Math.min((double) MAX_EXTENSION.pos, HORIZONTAL_EXPANSION_LIMIT/Math.cos(Math.min(approxCurrPivotInRadians, Math.PI/2-0.001)))
                    , 0);
            if (right_stick_y != 0) {
                int targetLeft = farPivotPIDMotor.getTarget() + (int) (right_stick_y * SPEED);
                int targetRight = closePivotPIDMotor.getTarget() + (int) (right_stick_y * SPEED);

                int clippedLeft, clippedRight;
                if (gamepad.gamepad.right_trigger > 0) {
                    clippedRight = Math.min(targetRight, MAX_EXTENSION.pos);
                    clippedLeft = Math.min(targetLeft, MAX_EXTENSION.pos);
                } else {
                    clippedRight = Range.clip(targetRight, MIN_EXTENSION.pos, MAX_EXTENSION.pos);
                    clippedLeft = Range.clip(targetLeft, MIN_EXTENSION.pos, MAX_EXTENSION.pos);
                    //NOT YET USING EFFECTIVE CURRENT MAX EXTENSION
                }

                farPivotPIDMotor.setTarget(clippedLeft);
                closePivotPIDMotor.setTarget(clippedRight);
            }

            telemetry.addData("approxCurrPivotInRadians", approxCurrPivotInRadians);
            telemetry.addData("effectiveCurrentMaxExtension", effectiveCurrentMaxExtension);
        }

        if (!magneticLimitSwitch.getState() && !limitSwitchWasActiveLastFrame){ //For some reason the actual output of the magnetic limit switch is reversed so I am reversing it here
            telemetry.addLine("Magnetic Limit Switch Activated");
            farPivotPIDMotor.ResetPID();
            closePivotPIDMotor.ResetPID();
            limitSwitchWasActiveLastFrame = true;
        } else {
            limitSwitchWasActiveLastFrame = false;
        }

        farPivotPIDMotor.update();
        closePivotPIDMotor.update();
        //telemetry.addData("farPivotPIDMotor Pos:", farPivotPIDMotor.getPos());
        //telemetry.addData("farPivotPIDMotor Target:", farPivotPIDMotor.getTarget());
        //telemetry.addData("farPivotPIDMotor Power:", farPivotPIDMotor.getPower());
        //telemetry.addData("closePivotPIDMotor Pos:", closePivotPIDMotor.getPos());
        //telemetry.addData("closePivotPIDMotor Target:", closePivotPIDMotor.getTarget());
        //telemetry.addData("closePivotPIDMotor Power:", closePivotPIDMotor.getPower());
    }

    @Override
    public void update() {

    }

    public void SetExtension(ExtensionPos extensionPos){
        farPivotPIDMotor.setTarget(extensionPos.pos);
        closePivotPIDMotor.setTarget(extensionPos.pos);
    }
}
