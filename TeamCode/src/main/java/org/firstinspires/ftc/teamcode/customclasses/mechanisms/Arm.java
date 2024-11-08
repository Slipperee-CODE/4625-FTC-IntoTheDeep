package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;

public class Arm extends Mechanism {
    public enum ArmState{
        DEFAULT(ArmPivoter.PivotPos.DEFAULT_PIVOT, ArmExtender.ExtensionPos.DEFAULT_EXTENSION),
        SUBMERSIBLE_GRAB(ArmPivoter.PivotPos.DEFAULT_PIVOT, ArmExtender.ExtensionPos.SUBMERSIBLE_EXTENSION),
        LOWER_BUCKET(ArmPivoter.PivotPos.LOWER_BUCKET_PIVOT, ArmExtender.ExtensionPos.LOWER_BUCKET_EXTENSION),
        UPPER_BUCKET(ArmPivoter.PivotPos.UPPER_BUCKET_PIVOT, ArmExtender.ExtensionPos.UPPER_BUCKET_EXTENSION),
        LOWER_BAR(ArmPivoter.PivotPos.LOWER_SPECIMEN_BAR_PIVOT, ArmExtender.ExtensionPos.LOWER_SPECIMEN_BAR_EXTENSION),
        UPPER_BAR(ArmPivoter.PivotPos.UPPER_SPECIMEN_BAR_PIVOT, ArmExtender.ExtensionPos.UPPER_SPECIMEN_BAR_EXTENSION),
        LOWER_HANG(ArmPivoter.PivotPos.LOWER_HANG_PIVOT, ArmExtender.ExtensionPos.LOWER_HANG_EXTENSION);

        ArmPivoter.PivotPos pivotPos;
        ArmExtender.ExtensionPos extensionPos;
        ArmState(ArmPivoter.PivotPos pivotPos, ArmExtender.ExtensionPos extensionPos) {
            this.pivotPos = pivotPos;
            this.extensionPos = extensionPos;
        }
    }

    private ArmPivoter armPivoter = null;
    private ArmExtender armExtender = null;
    public Claw claw = null;

    private boolean isSelectingEndPos = false;
    private boolean isBarSelected = false;

    public Arm(HardwareMap hardwareMap, CustomGamepad gamepad){
        this.gamepad = gamepad;
        armPivoter = new ArmPivoter(hardwareMap, gamepad);
        armExtender = new ArmExtender(hardwareMap, gamepad);
        claw = new Claw(hardwareMap, gamepad);
    }

    public Arm(HardwareMap hardwareMap){
        armPivoter = new ArmPivoter(hardwareMap);
        armExtender = new ArmExtender(hardwareMap);
        claw = new Claw(hardwareMap);
    }

    @Override
    public void update() {
        if (gamepad != null){
            if (!isSelectingEndPos){
                if (gamepad.yDown){
                    claw.triggerGamepadYClawMode();
                } else if (gamepad.bDown){
                    claw.triggerGamepadBClawMode();
                } else if (gamepad.aDown){
                    if (claw.triggerGamepadAClawMode()){
                        setArmState(ArmState.SUBMERSIBLE_GRAB);
                    }
                }
            } else {
                if (isBarSelected){
                    if (gamepad.upDown){
                        setArmState(ArmState.UPPER_BAR);
                    } else if (gamepad.downDown){
                        setArmState(ArmState.LOWER_BAR);
                    }
                } else {
                    if (gamepad.upDown){
                        setArmState(ArmState.UPPER_BUCKET);
                    } else if (gamepad.downDown){
                        setArmState(ArmState.LOWER_BUCKET);
                    }
                }

                if (gamepad.xDown){
                    isSelectingEndPos = false;
                    isBarSelected = false;
                    setArmState(ArmState.DEFAULT);
                }
            }

            if (gamepad.leftDown || gamepad.rightDown){
                isSelectingEndPos = true;
                if (gamepad.rightDown){
                    isBarSelected = true;
                } else if (gamepad.leftDown){
                    isBarSelected = false;
                }
            }
        }

        armPivoter.update();
        armExtender.update();
        claw.update();
    }

    @Override
    public void update(Telemetry telemetry) {
        update();
    }

    public void setArmState(ArmState armState){
        armPivoter.SetPivot(armState.pivotPos);
        armExtender.SetExtension(armState.extensionPos);
    }

    public class SetArmState implements Action {
        private ArmState armState;

        public SetArmState(ArmState armState){
            super();
            this.armState = armState;
        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            packet.addLine("Arm State has been set");
            setArmState(armState);
            return false; //Stops the method from looping
        }
    }
    public Action setArmStateAction(ArmState armState){ return new SetArmState(armState); }
}
