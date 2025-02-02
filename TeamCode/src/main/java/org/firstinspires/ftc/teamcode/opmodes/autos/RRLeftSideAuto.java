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
@Autonomous(name = "RRLeftSideAuto_2", group = "Autonomous")
public class RRLeftSideAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action initialTrajectory;
    private Action moveToFirstSamplePickup;
    private Action moveToSamplePlace1;
    private Action moveToSamplePlace2;
    private Action moveToSamplePlace3;

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
                .setTangent(Math.PI/2)
                .lineToY(-55)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToFirstSamplePickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.PI/2)
                .waitSeconds(1)
                .build();

        moveToSamplePlace1 = roadrunnerDrivetrain.actionBuilder(new Pose2d(-48,-48, Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToSecondSamplePickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-58, -48), Math.PI/2)
                .waitSeconds(1)
                .build();

        moveToSamplePlace2 = roadrunnerDrivetrain.actionBuilder(new Pose2d(-58,-48, Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToThirdSamplePickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-55, -26), Math.PI)
                .waitSeconds(1)
                .build();

        moveToSamplePlace3 = roadrunnerDrivetrain.actionBuilder(new Pose2d(-55,-26, Math.PI))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        park = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54, Math.PI/4))
                .splineToLinearHeading(new Pose2d(-26, -10, Math.PI), 0)
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
                                samplePickupSequenceAction(),
                                samplePlaceSequenceAction(moveToSamplePlace1, park),
                                new SleepAction(1),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.PARK)),
                                //samplePickupSequenceAction(),
                                //samplePlaceSequenceAction(moveToSamplePlace, moveToThirdSamplePickup),
                                //samplePickupSequenceAction(),
                                //samplePlaceSequenceAction(moveToSamplePlace, park),

                                new InstantAction(() -> arm.deactivatePIDMotors()) //SUPER IMPORTANT LINE BECAUSE IT PREVENTS AN INFINITE LOOP WHEN STOPPED
                        ))
                );
    }

    @Override
    public void stop(){
        arm.deactivatePIDMotors();

    }

    private Action samplePlaceSequenceAction(Action enterTrajectory, Action exitTrajectory) {
        return new SequentialAction(
                enterTrajectory,
                arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                new SleepAction(.5),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                new SleepAction(2),
                arm.preSampleDeposit(),
                new SleepAction(1),
                arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                new SleepAction(.25),
                arm.claw.setClawState(RRClaw.ClawPos.PRE_SPECIMEN_GRAB),
                new SleepAction(1),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),
                new SleepAction(1),

                new ParallelAction(
                        exitTrajectory,
                        new SequentialAction(
                                new SleepAction(1),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.LOWER_BAR)),
                                new SleepAction(.25),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                                new SleepAction(.5),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.DEFAULT))
                        )

                )
        );
    }

    private Action samplePickupSequenceAction(){
        return new SequentialAction(
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB)),
                new SleepAction(2),
            arm.setupForSampleGrab(0),
            new SleepAction(1),
            arm.grabSample(),
            new SleepAction(1),
            new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT))
        );
    }
    /**
     * Run [a] to completion in a blocking loop.
     */


}

