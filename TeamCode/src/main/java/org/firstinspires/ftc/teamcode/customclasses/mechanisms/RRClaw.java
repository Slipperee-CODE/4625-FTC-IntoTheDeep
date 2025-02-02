package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRMechanism;

import java.util.List;

public class RRClaw extends RRMechanism {
    public enum ClawPos{
        PRE_SPECIMEN_GRAB(SPECIMEN_GRAB_WRIST_POS, CLAW_NOT_GRABBING_POS, 1), //third parameter was 0
        SPECIMEN_GRAB(SPECIMEN_GRAB_WRIST_POS, CLAW_GRABBING_POS,1), //third parameter was 0
        PRE_SAMPLE_GRAB(SUBMERSIBLE_GRAB_WRIST_POS, CLAW_NOT_GRABBING_POS),
        SAMPLE_GRAB(SUBMERSIBLE_GRAB_WRIST_POS, CLAW_GRABBING_POS, -1),
        POST_GRAB(POST_GRAB_WRIST_POS, CLAW_GRABBING_POS),
        RESET(STOWED_WRIST_POS, CLAW_NOT_GRABBING_POS), //Also works as RELEASE_SPECIMEN

        RELEASE_SAMPLE(SAMPLE_DEPOSIT_WRIST_POS, CLAW_NOT_GRABBING_POS),


        PRE_SAMPLE_DEPOSIT(SAMPLE_DEPOSIT_WRIST_POS, CLAW_GRABBING_POS),
        PRE_SPECIMEN_DEPOSIT(SUBMERSIBLE_GRAB_WRIST_POS, CLAW_GRABBING_POS);

        float wristServoPos, clawServoPos, rotationServoPos;
        ClawPos(float wristServoPos, float clawServoPos, float rotationServoPos) {
            this.wristServoPos = wristServoPos;
            this.clawServoPos = clawServoPos;
            this.rotationServoPos = rotationServoPos;
        }

        ClawPos(float wristServoPos, float clawServoPos) {
            this.wristServoPos = wristServoPos;
            this.clawServoPos = clawServoPos;
            this.rotationServoPos = 1.0f;
        }
    }

    private static final float STOWED_WRIST_POS = .66f;
    private static final float SAMPLE_DEPOSIT_WRIST_POS = 0.35f;
    private static final float POST_GRAB_WRIST_POS = 0.45f;

    private static final float SUBMERSIBLE_GRAB_WRIST_POS = 0.0f;
    private static final float SPECIMEN_GRAB_WRIST_POS = 0.35f; //was .35f for all other meets || was .4 for Meet 2 || THIS MIGHT NEED TO BE CHANGED FOR OFF WALL GRABBING

    private static final float CLAW_NOT_GRABBING_POS = 0.15f;
    private static final float CLAW_GRABBING_POS = 0.45f; //MAKE THIS MOREEEE

    private final Servo wristServo;
    private final Servo clawServo;
    public final Servo rotationServo;

    private ClawPos currentClawPos;

    public RRClaw(HardwareMap hardwareMap, CustomGamepad gamepad){
        this(hardwareMap);
        this.gamepad = gamepad;
    }

    public RRClaw(HardwareMap hardwareMap){
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rotationServo = hardwareMap.get(Servo.class, "rotationServo");
        wristServo.setPosition(STOWED_WRIST_POS);
        clawServo.setPosition(CLAW_NOT_GRABBING_POS);
        rotationServo.setPosition(1.0f);
    }

    public Action emulatedClawRotation(float gamepadLeftStickX) {
        return new InstantAction(() -> rotationServo.setPosition(Math.max(0, 1-gamepadLeftStickX)));
    }

    public Action setClawState(ClawPos clawPos){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                currentClawPos = clawPos;
                wristServo.setPosition(clawPos.wristServoPos);
                clawServo.setPosition(clawPos.clawServoPos);
                if (clawPos.rotationServoPos != -1){
                    rotationServo.setPosition(clawPos.rotationServoPos);
                }
                return false;
            }
        };
    }

    public void initUpdateForGrab(){
        if (gamepad.upDown){
            clawServo.setPosition(CLAW_GRABBING_POS);
            //rotationServo.setPosition(0);
        }
        if (gamepad.downDown){
            clawServo.setPosition(CLAW_NOT_GRABBING_POS);
            //rotationServo.setPosition(1);
        }
    }

    public ClawPos getCurrentClawPos() {
        return currentClawPos;
    }

    public boolean MatchesCurrentClawPos(ClawPos clawPos){
        return this.currentClawPos.clawServoPos == clawPos.clawServoPos
                && this.currentClawPos.wristServoPos == clawPos.wristServoPos
                && this.currentClawPos.rotationServoPos == clawPos.rotationServoPos;
    }

    @Override
    public void queueActions() {

    }

    @Override
    public Action queueUpdateActions() {
        return null;
    }
}

