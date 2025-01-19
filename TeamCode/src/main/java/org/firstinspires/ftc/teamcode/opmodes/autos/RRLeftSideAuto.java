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
@Autonomous(name = "RRBasicAuto", group = "Autonomous")
public class RRLeftSideAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action initialTrajectory;
    private Action moveToFirstSamplePickup;
    private Action moveToFirstSamplePlace;

    private Action moveToSecondSamplePickup;
    private Action moveToSecondSamplePlace;

    private Action moveToThirdSamplePickup;
    private Action moveToThirdSamplePlace;

    private Action park;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(9, -64, Math.PI/2));

        initialTrajectory = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYConstantHeading(-37)
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
                                //specimenPlaceSequenceAction(moveToSpecimenPlace, moveToSpecimenPickup),
                                //specimenPickupSequenceAction(),

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

