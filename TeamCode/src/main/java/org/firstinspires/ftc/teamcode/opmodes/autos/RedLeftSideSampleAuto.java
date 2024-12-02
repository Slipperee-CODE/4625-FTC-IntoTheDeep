package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.WaitingAuto;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRClaw;

@Config
@Autonomous(name = "RedLeftSideSampleAuto", group = "Autonomous")
public class RedLeftSideSampleAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action initialTrajectory;
    private Action moveToFirstSamplePickup;
    private Action moveToSecondSamplePickup;
    private Action moveToThirdSamplePickup;

    private Action moveToFirstSamplePlace;
    private Action moveToSecondSamplePlace;
    private Action moveToThirdSamplePlace;

    private Action park;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, runningActions, gamepad2);

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-9, -64, Math.PI/2));

        initialTrajectory = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYConstantHeading(-46)
                .build();
        moveToFirstSamplePickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-48, -46, Math.PI/2), 0)
                .build();
        moveToFirstSamplePlace = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .build();
        moveToSecondSamplePickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-58, -46, Math.PI/2), 0)
                .build();
        moveToSecondSamplePlace = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .build();
        moveToThirdSamplePickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-58, -46, 2*Math.PI/3), 0)
                .build();
        moveToThirdSamplePlace = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .build();
        park = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-24, 8, 0),0)
                .build();
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
                new SequentialAction(
                    new ParallelAction(
                        arm.setArmState(RRArm.ArmState.UPPER_BAR),
                        initialTrajectory
                    ),
                    arm.claw.setClawState(RRClaw.ClawPos.RESET)
                )
        );
    }
}

