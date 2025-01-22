package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.WaitingAuto;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRClaw;

@Config
@Autonomous(name = "RRLeftSideAuto", group = "Autonomous")
public class RRLeftSideAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action initialTrajectory;
    private Action moveToFirstSamplePickup;
    private Action moveToSamplePlace;
    private Action moveToSecondSamplePickup;
    private Action moveToThirdSamplePickup;
    private Action park;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-9, -64, Math.PI/2));

        initialTrajectory = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.PI/4)
                .build();

        moveToFirstSamplePickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-48, -40, Math.PI/2), 0)
                .waitSeconds(1)
                .build();

        moveToSecondSamplePickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-58, -40, Math.PI/2), 0)
                .waitSeconds(1)
                .build();

        moveToThirdSamplePickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-55, -26), Math.PI)
                .waitSeconds(1)
                .build();

        moveToSamplePlace = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.PI/4)
                .build();

        park = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(-24, -10, Math.PI), 0)
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
                new ParallelAction(
                        arm.queueUpdateActions(),
                        new SequentialAction(
                                arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                                samplePlaceSequenceAction(initialTrajectory, moveToFirstSamplePickup),
                                //samplePickupSequenceAction(),
                                //samplePlaceSequenceAction(moveToSamplePlace, moveToSecondSamplePickup),
                                //samplePickupSequenceAction(),
                                //samplePlaceSequenceAction(moveToSamplePlace, moveToThirdSamplePickup),
                                //samplePickupSequenceAction(),
                                //samplePlaceSequenceAction(moveToSamplePlace, park),

                                new InstantAction(() -> arm.deactivatePIDMotors()) //SUPER IMPORTANT LINE BECAUSE IT PREVENTS AN INFINITE LOOP WHEN STOPPED
                        )
                )
        );
    }

    @Override
    public void stop(){
        arm.deactivatePIDMotors();
    }

    private Action samplePlaceSequenceAction(Action enterTrajectory, Action exitTrajectory) {
        return new SequentialAction(
                new ParallelAction(
                        enterTrajectory,
                        new SequentialAction(
                                new SleepAction(1),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                                arm.preSampleDeposit()
                        )
                ),

                new SleepAction(1),
                arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                new SleepAction(1),

                //Reduce Extension Here

                new ParallelAction(
                        exitTrajectory,
                        new SequentialAction(
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.LOWER_BAR)),
                                new SleepAction(1),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.DEFAULT))
                        )

                )
        );
    }

    private Action samplePickupSequenceAction(){
        return new SequentialAction(
            arm.setupForSampleGrab(0.5f),
            new SleepAction(1),
            arm.grabSample()
        );
    }
}

