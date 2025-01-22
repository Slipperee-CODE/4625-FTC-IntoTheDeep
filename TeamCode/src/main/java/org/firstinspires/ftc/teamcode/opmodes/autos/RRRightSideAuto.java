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
@Autonomous(name = "RRRightSideAuto", group = "Autonomous")
public class RRRightSideAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action initialTrajectory;
    private Action moveToFirstSpecimenPickup;
    private Action moveToSpecimenPickup;
    private Action moveToSpecimenPlace;
    private Action park;

    private static final int SPECIMEN_CYCLES = 3; //Doesn't do anything right now

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(9, -64, -Math.PI/2));

        initialTrajectory = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYConstantHeading(-37)
                .build();

        moveToFirstSpecimenPickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeTo(new Vector2d(30,-40))
                .splineToLinearHeading(new Pose2d(42, -12, -Math.PI/2), 0)
                .lineToX(42.5)
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(52, -9, -Math.PI/2), 0)
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(57, -9, -Math.PI/2), 0)
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .strafeToLinearHeading(new Vector2d(37, -46), -Math.PI/2)
                .build();

        moveToSpecimenPlace = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(9, -34, -Math.PI/2), Math.PI)
                .build();

        moveToSpecimenPickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(37, -46, -Math.PI/2), -Math.PI/2)
                .build();

        park = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .splineToLinearHeading(new Pose2d(37, -50, -Math.PI/2), -Math.PI/2)
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
                                specimenPlaceSequenceAction(initialTrajectory, moveToFirstSpecimenPickup),
                                //specimenPickupSequenceAction(),
                                //specimenPlaceSequenceAction(moveToSpecimenPlace, moveToSpecimenPickup),
                                //specimenPickupSequenceAction(),
                                //specimenPlaceSequenceAction(moveToSpecimenPlace, moveToSpecimenPickup),
                                //specimenPickupSequenceAction(),
                                //specimenPlaceSequenceAction(moveToSpecimenPlace, park),

                                new InstantAction(() -> arm.deactivatePIDMotors()) //SUPER IMPORTANT LINE BECAUSE IT PREVENTS AN INFINITE LOOP WHEN STOPPED
                        )
                )
        );
    }

    @Override
    public void stop(){
        arm.deactivatePIDMotors();
    }

    private Action specimenPlaceSequenceAction(Action enterTrajectory, Action exitTrajectory) {
        return new SequentialAction(
                new ParallelAction(
                        enterTrajectory,
                        new SequentialAction(
                                new SleepAction(1),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BAR)),
                                arm.preSpecimenDeposit()
                        )
                ),

                new SleepAction(1),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BAR)),
                new SleepAction(1),
                arm.claw.setClawState(RRClaw.ClawPos.RESET),
                new SleepAction(1),

                //Lower Extension Here

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

        private Action specimenPickupSequenceAction(){
            return new SequentialAction(
                arm.setupForSpecimenGrab(),
                new SleepAction(1),
                arm.grabSpecimen()
            );
    }
}

