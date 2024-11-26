package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;

public class Claw extends Mechanism {
    private static final float STOWED_WRIST_POS = .7f;
    private static final float STOWED_WITH_SAMPLE_WRIST_POS = 0.5f;
    private static final float STOWED_WITH_SPECIMEN_WRIST_POS = 0.5f;

    private static final float SUBMERSIBLE_GRAB_WRIST_POS = 0.0f;
    private static final float SPECIMEN_GRAB_WRIST_POS = 0.35f; //was .4 for Meet 2

    private static final float CLAW_NOT_GRABBING_POS = 0.1f;
    private static final float CLAW_GRABBING_POS = 0.295f;

    private static final float ROTATIONAL_LIMIT = 0.5f;

    private final Servo wristServo;
    private final Servo clawServo;
    public final Servo rotationServo;

    private boolean isGamepadYClawModeActive = false;
    private boolean isGamepadAClawModeActive = false;

    public Claw(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public Claw(HardwareMap hardwareMap){
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rotationServo = hardwareMap.get(Servo.class, "rotationServo");
        wristServo.setPosition(STOWED_WRIST_POS);
        clawServo.setPosition(CLAW_NOT_GRABBING_POS);
    }

    @Override
    public void update() {
        if (isGamepadAClawModeActive) {
            //rotationServo.setPosition(gamepad.right_stick_x * ROTATIONAL_LIMIT);
        }
    }

    @Override
    public void update(Telemetry telemetry) {
        update();
    }

    public boolean triggerGamepadYClawMode(){
        if (isGamepadAClawModeActive) return false;

        if (isGamepadYClawModeActive){
            isGamepadYClawModeActive = false;
            clawServo.setPosition(CLAW_GRABBING_POS);
            wristServo.setPosition(STOWED_WITH_SPECIMEN_WRIST_POS);

        } else {
            isGamepadYClawModeActive = true;
            clawServo.setPosition(CLAW_NOT_GRABBING_POS);
            wristServo.setPosition(SPECIMEN_GRAB_WRIST_POS);
        }

        return isGamepadYClawModeActive;
    }

    public void triggerGamepadBClawMode(){
        clawServo.setPosition(CLAW_NOT_GRABBING_POS);
        wristServo.setPosition(STOWED_WRIST_POS);
        //rotationServo.setPosition(0.5);
    }

    public boolean triggerGamepadAClawMode(){
        if (isGamepadYClawModeActive) return false;

        if (isGamepadAClawModeActive){
            isGamepadAClawModeActive = false;
            clawServo.setPosition(CLAW_GRABBING_POS);
            wristServo.setPosition(STOWED_WITH_SAMPLE_WRIST_POS);
        } else {
            isGamepadAClawModeActive = true;
            //The below setPositions need to be delayed to allow for reach to extend over bar, same with retraction but reversed
            //This is fine because we can pull up real close and activate it there, the servo should come down after it is in a safe area
            clawServo.setPosition(CLAW_NOT_GRABBING_POS);
            wristServo.setPosition(SUBMERSIBLE_GRAB_WRIST_POS);
        }

        return isGamepadAClawModeActive;
    }

    public void initUpdateForGrab(){
        if (gamepad.yDown){
            clawServo.setPosition(CLAW_GRABBING_POS);
        }
        if (gamepad.bDown){
            clawServo.setPosition(CLAW_NOT_GRABBING_POS);
        }
    }
}

