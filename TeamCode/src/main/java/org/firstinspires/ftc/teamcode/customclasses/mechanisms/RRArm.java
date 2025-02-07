package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRMechanism;

import java.util.List;

public class RRArm extends RRMechanism {
    public enum ArmState {
        DEFAULT(RRArmPivoter.PivotPos.DEFAULT_PIVOT, RRArmExtender.ExtensionPos.DEFAULT_EXTENSION),
        SAFE_DEFAULT(RRArmPivoter.PivotPos.SAFE_DEFAULT_PIVOT, RRArmExtender.ExtensionPos.DEFAULT_EXTENSION),
        AUTO_SAFE_DEFAULT(RRArmPivoter.PivotPos.SAFE_DEFAULT_PIVOT, RRArmExtender.ExtensionPos.LOWER_BUCKET_EXTENSION),
        PARK(RRArmPivoter.PivotPos.UPPER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.LOWER_BUCKET_EXTENSION),
        SUBMERSIBLE_GRAB(RRArmPivoter.PivotPos.DEFAULT_PIVOT, RRArmExtender.ExtensionPos.SUBMERSIBLE_EXTENSION),
        LOWER_BUCKET(RRArmPivoter.PivotPos.LOWER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.LOWER_BUCKET_EXTENSION),
        UPPER_BUCKET(RRArmPivoter.PivotPos.UPPER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.UPPER_BUCKET_EXTENSION),
        LOWER_BAR(RRArmPivoter.PivotPos.LOWER_SPECIMEN_BAR_PIVOT, RRArmExtender.ExtensionPos.LOWER_SPECIMEN_BAR_EXTENSION),
        UPPER_BAR(RRArmPivoter.PivotPos.UPPER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.UPPER_SPECIMEN_BAR_EXTENSION), //Changed pivot to UPPER_BUCKET_PIVOT on purpose
        LOWER_HANG(RRArmPivoter.PivotPos.LOWER_HANG_PIVOT, RRArmExtender.ExtensionPos.LOWER_HANG_EXTENSION),
        WALL_GRAB(RRArmPivoter.PivotPos.WALL_GRAB_PIVOT, RRArmExtender.ExtensionPos.WALL_GRAB_EXTENSION),

        AUTO_SPECIMEN_PLACE_UPPER_BAR(RRArmPivoter.PivotPos.UPPER_SPECIMEN_BAR_PIVOT, RRArmExtender.ExtensionPos.AUTO_SPECIMEN_PLACE_UPPER_BAR),
        AUTO_SPECIMEN_PLACE_UPPER_BUCKET(RRArmPivoter.PivotPos.UPPER_BUCKET_PIVOT, RRArmExtender.ExtensionPos.AUTO_SAMPLE_PLACE_UPPER_BUCKET),
        AUTO_SAMPLE_GRAB(RRArmPivoter.PivotPos.SAFE_DEFAULT_PIVOT, RRArmExtender.ExtensionPos.AUTO_SAMPLE_GRAB),
        AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY(RRArmPivoter.PivotPos.UPPER_BUCKET_PIVOT_BUT_MORE_FORWARD, RRArmExtender.ExtensionPos.AUTO_SAFE_EXTENSION_HEIGHT);


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

    public RRArm(HardwareMap hardwareMap, CustomGamepad gamepad) {
        this.gamepad = gamepad;
        armPivoter = new RRArmPivoter(hardwareMap, gamepad);
        armExtender = new RRArmExtender(hardwareMap, gamepad);
        claw = new RRClaw(hardwareMap, gamepad);
    }

    public List<Action> queueActions(List<Action> runningActions, Telemetry telemetry) {
        if (!isSelectingEndPos) {
            if (gamepad.yDown) {
                    if (gamepad.yToggle) {
                        runningActions.add(
                                setupForSpecimenGrab()
                        );
                    } else {
                        runningActions.add(
                                grabSpecimen()
                        );
                    }
            }
            else if (gamepad.bDown) {
                if (claw.MatchesCurrentClawPos(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT)){
                    runningActions.add(
                            sampleDeposit()
                    );
                } else {
                    runningActions.add(claw.setClawState(RRClaw.ClawPos.RESET));
                }
            }
            else if (gamepad.aDown) {
                if (gamepad.aToggle) {
                    runningActions.add(
                            setupForSampleGrab(1.0f)
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
                    setArmState(ArmState.UPPER_BAR);
                    isSelectingEndPos = false;
                    runningActions.add(
                            preSpecimenDeposit()
                    );
                } else if (gamepad.downDown) {
                    setArmState(ArmState.LOWER_BAR);
                    isSelectingEndPos = false;
                }
            } else {
                if (gamepad.upDown) {
                    setArmState(ArmState.UPPER_BUCKET);
                    isSelectingEndPos = false;
                    runningActions.add(
                            preSampleDeposit()
                    );
                } else if (gamepad.downDown) {
                    setArmState(ArmState.LOWER_BUCKET);
                    isSelectingEndPos = false;
                    runningActions.add(
                            preSampleDeposit()
                    );
                }
            }
        }

        if (gamepad.xDown) {
            isSelectingEndPos = false;
            isBarSelected = false;
            if (gamepad.gamepad.left_trigger > 0) {
                setArmState(ArmState.DEFAULT);
            } else {
                runningActions.add(
                        new SequentialAction(
                                new InstantAction(() -> setArmState(ArmState.UPPER_BUCKET)),
                                new SleepAction(5.0f),
                                new InstantAction(() -> setArmState(ArmState.SAFE_DEFAULT))
                        )
                );

            }
        }

        if (gamepad.gamepad.left_bumper){ //manual pivot override
            armPivoter.leftPivotPIDMotor.setTarget(armPivoter.leftPivotPIDMotor.getTarget()-5);
            armPivoter.rightPivotPIDMotor.setTarget(armPivoter.rightPivotPIDMotor.getTarget()-5);
        }  else if (gamepad.gamepad.right_bumper){
            armPivoter.leftPivotPIDMotor.setTarget(armPivoter.leftPivotPIDMotor.getTarget()+5);
            armPivoter.rightPivotPIDMotor.setTarget(armPivoter.rightPivotPIDMotor.getTarget()+5);
        }

        if (gamepad.leftDown || gamepad.rightDown) {
            isSelectingEndPos = true;
            if (gamepad.rightDown) {
                isBarSelected = true;
            } else if (gamepad.leftDown) {
                isBarSelected = false;
            }
        }
        if (gamepad.aToggle) {
            runningActions.add(claw.emulatedClawRotation(gamepad.left_stick_x));
        }
        armExtender.update(armPivoter.GetCurrPivotInRadians());
        return runningActions;
    }

    public void setArmState(ArmState armState) {
        CURR_STATE = armState;
        armPivoter.SetPivot(armState.pivotPos);
        armExtender.SetExtension(armState.extensionPos);
    }

    @Override
    public void queueActions() {

    }

    public Action queueUpdateActions(){
        return new ParallelAction(
            armPivoter.queueUpdateActions(),
            armExtender.queueUpdateActions()
        );
    }

    public void deactivatePIDMotors(){
        armPivoter.deactivate();
        armExtender.deactivate();
    }

    public Action setupForSpecimenGrab(){
        return new ParallelAction(
                claw.setClawState(RRClaw.ClawPos.PRE_SPECIMEN_GRAB)
                //new InstantAction(() -> setArmState(ArmState.WALL_GRAB))
        );
    }

    public Action grabSpecimen(){
        return new SequentialAction(
                claw.setClawState(RRClaw.ClawPos.SPECIMEN_GRAB),
                new SleepAction(0.25),
                claw.setClawState(RRClaw.ClawPos.POST_GRAB)
        );
    }

    public Action setupForSampleGrab(float rotationServoPos){
        return new ParallelAction(
                claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB)
                //claw.emulatedClawRotation(rotationServoPos)
        );
    }

    public Action grabSample(){
        return new SequentialAction(
                claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                new SleepAction(0.25),
                claw.setClawState(RRClaw.ClawPos.POST_GRAB)
        );
    }

    public Action preSampleDeposit(){
        return new SequentialAction(
                claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT)
        );
    }

    public Action sampleDeposit(){
        return new SequentialAction(
            claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
            new SleepAction(.25),
            claw.setClawState(RRClaw.ClawPos.POST_GRAB) //Used to be POST_RELEASE_SAMPLE
        );
    }

    public Action preSpecimenDeposit(){
        return new SequentialAction(
                claw.setClawState(RRClaw.ClawPos.PRE_SPECIMEN_DEPOSIT)
        );
    }
}