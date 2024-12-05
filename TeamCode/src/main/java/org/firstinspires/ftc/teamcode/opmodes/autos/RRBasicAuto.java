package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.WaitingAuto;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRClaw;

@Config
@Autonomous(name = "RRBasicAuto", group = "Autonomous")
public class RRBasicAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action initialTrajectory;
    private Action moveToFirstSpecimenPickup;
    private Action moveToSecondSpecimenPickup;
    private Action moveToThirdSpecimenPickup;

    private Action moveToFirstSpecimenPlace;
    private Action moveToSecondSpecimenPlace;
    private Action moveToThirdSpecimenPlace;

    private Action park;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, runningActions, gamepad2);

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-9, -64, Math.PI/2));
    }

    @Override
    public void init_loop() {
        super.init_loop();
        gamepad2.update();
        arm.claw.initUpdateForGrab();
    }

    @Override
    protected void startAfterWait() {
        Actions.runBlocking(
                new ParallelAction(
                        arm.queueUpdateActions(),
                        new SequentialAction(
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BAR)),
                                initialTrajectory,
                                arm.claw.setClawState(RRClaw.ClawPos.RESET)
                                //new InstantAction(() -> arm.setArmState(RRArm.ArmState.DEFAULT)),
                                //moveToFirstSamplePickup,
                                //arm.setupForSampleGrab(0.5f)
                        )
                )
        );
    }
}

