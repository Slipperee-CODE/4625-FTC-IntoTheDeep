package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRMechanism;

import java.util.List;

public class RRArm extends RRMechanism {
    public enum ArmState {
        DEFAULT(RRArmPivoter.PivotPos.DEFAULT_PIVOT, RRArmExtender.ExtensionPos.DEFAULT_EXTENSION),
        SAFE_DEFAULT(RRArmPivoter.PivotPos.SAFE_DEFAULT_PIVOT, RRArmExtender.ExtensionPos.DEFAULT_EXTENSION),
        SUBMERSIBLE_GRAB(RRArmPivoter.PivotPos.DEFAULT_PIVOT, RRArmExtender.ExtensionPos.SUBMERSIBLE_EXTENSION),
        LOWER_BUCKET(RRArmPivoter.PivotPos.LOWER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.LOWER_BUCKET_EXTENSION),
        UPPER_BUCKET(RRArmPivoter.PivotPos.UPPER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.UPPER_BUCKET_EXTENSION),
        LOWER_BAR(RRArmPivoter.PivotPos.LOWER_SPECIMEN_BAR_PIVOT, RRArmExtender.ExtensionPos.LOWER_SPECIMEN_BAR_EXTENSION),
        UPPER_BAR(RRArmPivoter.PivotPos.UPPER_SPECIMEN_BAR_PIVOT, RRArmExtender.ExtensionPos.UPPER_SPECIMEN_BAR_EXTENSION),
        LOWER_HANG(RRArmPivoter.PivotPos.LOWER_HANG_PIVOT, RRArmExtender.ExtensionPos.LOWER_HANG_EXTENSION),
        WALL_GRAB(RRArmPivoter.PivotPos.WALL_GRAB_PIVOT, RRArmExtender.ExtensionPos.WALL_GRAB_EXTENSION);

        RRArmPivoter.PivotPos pivotPos;
        RRArmExtender.ExtensionPos extensionPos;

        ArmState(RRArmPivoter.PivotPos pivotPos, RRArmExtender.ExtensionPos extensionPos) {
            this.pivotPos = pivotPos;
            this.extensionPos = extensionPos;
        }
    }

    private ArmState CURR_STATE = ArmState.DEFAULT;

    public RRArmPivoter armPivoter = null;
    public RRArmExtender armExtender = null;
    public RRClaw claw = null;

    private boolean isSelectingEndPos = false;
    private boolean isBarSelected = false;

    public RRArm(HardwareMap hardwareMap, List<Action> runningActions, CustomGamepad gamepad) {
        this.gamepad = gamepad;
        this.runningActions = runningActions;
        armPivoter = new RRArmPivoter(hardwareMap, runningActions, gamepad);
        armExtender = new RRArmExtender(hardwareMap, runningActions, gamepad);
        claw = new RRClaw(hardwareMap, runningActions, gamepad);
    }

    public RRArm(HardwareMap hardwareMap) {
        armPivoter = new RRArmPivoter(hardwareMap);
        armExtender = new RRArmExtender(hardwareMap);
        claw = new RRClaw(hardwareMap);
    }

    public void queueActions() {
        if (gamepad != null) {
            if (!isSelectingEndPos) {
                if (gamepad.yDown) {
                    if (CURR_STATE == ArmState.LOWER_BAR || CURR_STATE == ArmState.UPPER_BAR || CURR_STATE == ArmState.LOWER_BUCKET || CURR_STATE == ArmState.UPPER_BUCKET) {
                        //runningActions.add(PlaceSample/Specimen Action)
                    } else {
                        if (gamepad.yDown){
                            if (!gamepad.yToggle){
                                runningActions.add(
                                        setupForSpecimenGrab()
                                );
                            } else {
                                runningActions.add(
                                        grabSpecimen()
                                );
                            }
                        }
                    }
                } else if (gamepad.bDown) {
                    runningActions.add(claw.setClawState(RRClaw.ClawPos.RESET));
                } else if (gamepad.aDown) {
                    if (!gamepad.aToggle){
                        runningActions.add(
                                setupForSampleGrab(0.5f)
                        );
                    } else {
                        runningActions.add(
                            grabSample()
                        );
                    }
                }
            } else {
                if (isBarSelected) {
                    if (gamepad.upDown) {
                        runningActions.add(setArmState(ArmState.UPPER_BAR));
                        isSelectingEndPos = false;
                    } else if (gamepad.downDown) {
                        runningActions.add(setArmState(ArmState.LOWER_BAR));
                        isSelectingEndPos = false;
                    }
                } else {
                    if (gamepad.upDown) {
                        runningActions.add(setArmState(ArmState.UPPER_BUCKET));
                        isSelectingEndPos = false;
                    } else if (gamepad.downDown) {
                        runningActions.add(setArmState(ArmState.LOWER_BUCKET));
                        isSelectingEndPos = false;
                    }
                }
            }

            if (gamepad.xDown) {
                isSelectingEndPos = false;
                isBarSelected = false;
                if (gamepad.gamepad.left_trigger > 0) {
                    runningActions.add(setArmState(ArmState.DEFAULT));
                } else {
                    runningActions.add(setArmState(ArmState.SAFE_DEFAULT));
                }
            }

            if (gamepad.leftDown || gamepad.rightDown) {
                isSelectingEndPos = true;
                if (gamepad.rightDown) {
                    isBarSelected = true;
                } else if (gamepad.leftDown) {
                    isBarSelected = false;
                }
            }
            if (gamepad.aToggle){ runningActions.add(claw.emulatedClawRotation(gamepad.left_stick_x)); }
        }
    }

    public Action setArmState(ArmState armState) {
        CURR_STATE = armState;
        return new ParallelAction(
                armPivoter.SetPivot(armState.pivotPos),
                armExtender.SetExtension(armState.extensionPos)
        );
    }

    public Action setupForSpecimenGrab(){
        return new ParallelAction(
                claw.setClawState(RRClaw.ClawPos.PRE_SPECIMEN_GRAB),
                setArmState(ArmState.WALL_GRAB)
        );
    }

    public Action grabSpecimen(){
        return new SequentialAction(
                claw.setClawState(RRClaw.ClawPos.SPECIMEN_GRAB),
                new SleepAction(0.25),
                claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                setArmState(ArmState.DEFAULT)
        );
    }

    public Action setupForSampleGrab(float rotationServoPos){
        return new ParallelAction(
                claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB),
                claw.emulatedClawRotation(rotationServoPos)
        );
    }

    public Action grabSample(){
        return new SequentialAction(
                claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                new SleepAction(0.25),
                claw.setClawState(RRClaw.ClawPos.POST_GRAB)
        );
    }
}