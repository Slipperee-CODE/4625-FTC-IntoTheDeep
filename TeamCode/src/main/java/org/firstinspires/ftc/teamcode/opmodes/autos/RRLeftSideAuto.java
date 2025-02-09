package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Clock;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.WaitingAuto;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRClaw;

@Config
@Autonomous(name = "RRLeftSideAuto", group = "Autonomous")
public class RRLeftSideAuto extends WaitingAuto {
    private RRArm arm;
    private CustomGamepad gamepad2;

    private Action moveToPreSample1Place;
    private Action moveToSample1Place;
    private Action moveToSample2Pickup;

    private Action moveToPreSample2Place;
    private Action moveToSample2Place;
    private Action moveToSample3Pickup;

    private Action moveToPreSample3Place;
    private Action moveToSample3Place;
    private Action moveToSample4Pickup;

    private Action moveToPreSample4Place;
    private Action moveToSample4Place;
    private Action park;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);


        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-38, -64, Math.PI/2));


        moveToPreSample1Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .setTangent(Math.PI/2)
                .lineToY(-55)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample1Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToSample2Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.PI/2)
                .build();


        moveToPreSample2Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-48,-48,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample2Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToSample3Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-58, -48), Math.PI/2)
                .build();


        moveToPreSample3Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-58,-48,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample3Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();
        
        park = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
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
    protected void update(){
        arm.queueActions(telemetry);
        runActions();
        telemetry.update();
    }

    @Override
    protected void startAfterWait() {
        runningActions.add(
             new SequentialAction(
                     new ParallelAction(
                             arm.preSampleDeposit(),
                             moveToPreSample1Place
                     ),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                     new SleepAction(0.5f),
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                     new SleepAction(1),

                     moveToSample1Place,

                     arm.sampleDeposit(),

                     new SleepAction(.25),

                     arm.setupForSampleGrab(1.0f),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                     new SleepAction(2),
                     moveToSample2Pickup,
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB)),
                     new SleepAction(2),

                     arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                     new SleepAction(0.25),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                     new SleepAction(0.25),

                     new ParallelAction(
                             arm.preSampleDeposit(),
                             new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                             moveToPreSample2Place
                     ),

                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                    new SleepAction(0.5f),
                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                    new SleepAction(1),

                    moveToSample2Place,

                    arm.sampleDeposit(),

                    new SleepAction(.25),

                    arm.setupForSampleGrab(1.0f),

                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                    new SleepAction(2),
                    moveToSample3Pickup,
                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB)),
                    new SleepAction(2),

                    arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                    new SleepAction(0.25),
                    arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                    new SleepAction(0.25),

                    new ParallelAction(
                            arm.preSampleDeposit(),
                            new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                            moveToPreSample3Place
                    ),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                     new SleepAction(0.5f),
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                     new SleepAction(1),

                     moveToSample3Place,

                     arm.sampleDeposit(),

                     new SleepAction(.25),

                     arm.setupForSampleGrab(1.0f),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),
                     new SleepAction(2),
                     new ParallelAction(
                             arm.claw.setClawState(RRClaw.ClawPos.RESET),
                             new InstantAction(() -> arm.setArmState(RRArm.ArmState.SAFE_DEFAULT))
                     )

                )
        );
    }
}