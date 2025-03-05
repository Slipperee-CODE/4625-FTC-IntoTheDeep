package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
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

    private Action moveToPreSample1Place;
    private Action moveToSample1Place;

    private Action moveToPreSample2Pickup;
    private Action moveToSample2Pickup;
    private Action moveToPreSample2Place;
    private Action moveToSample2Place;

    private Action moveToPreSample3Pickup;
    private Action moveToSample3Pickup;
    private Action moveToPreSample3Place;
    private Action moveToSample3Place;

    private Action moveToPreSample4Pickup;
    private Action moveToSample4Pickup;
    private Action moveToPreSample4Place;
    private Action moveToSample4Place;

    private Action park;

    private boolean shouldUpdatePIDMotors = true;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);


        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-38, -64, Math.PI/2));
        //update sample place 1 and sample place 2 to be closer to bucket

        moveToPreSample1Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .setTangent(Math.PI/2)
                .lineToY(-55)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample1Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-53, -55), Math.PI/4)
                .build();


        moveToPreSample2Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-53,-55,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-48.5, -50), Math.PI/2)
                .build();

        moveToSample2Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-48.5,-50,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-48.5, -45), Math.PI/2)
                .build();

        moveToPreSample2Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-48.5,-45,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample2Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-53.5, -55.5), Math.PI/4)
                .build();


        moveToPreSample3Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-53.5,-55.5,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-58, -50), Math.PI/2)
                .build();

        moveToSample3Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-58,-50,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-58, -45), Math.PI/2)
                .build();

        moveToPreSample3Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-58,-45,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample3Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-55, -55.75), Math.PI/4)
                .build();

        //needs to be implemented into the auto trajectory below
        moveToPreSample4Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-55,-55.75,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-53, -43), Math.toRadians(120)) //move more left and more back
                .build();

        moveToSample4Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-53,-43,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(-53.5, -41), Math.toRadians(120)) //move more more left (slightly) and y=-43
                .build(); //add placement of fourth sample

        moveToPreSample4Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-51,-41,Math.toRadians(120)))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample4Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-56, -55.75), Math.PI/4)
                .build();


        park = roadrunnerDrivetrain.actionBuilder(new Pose2d(-56,-55.75,Math.PI/4))
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
        if (shouldUpdatePIDMotors){
            arm.queueActions(telemetry);
        } else {
            arm.armPivoter.rightPivotPIDMotor.setRawPower(0);
            arm.armPivoter.leftPivotPIDMotor.setRawPower(0);
            arm.armExtender.closePivotPIDMotor.setRawPower(0);
            arm.armExtender.farPivotPIDMotor.setRawPower(0);
        }
        runActions();
        telemetry.update();
    }

    @Override
    protected void startAfterWait() {
        runningActions.add(
             new SequentialAction(
                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT),
                     moveToPreSample1Place,

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                     new SleepAction(0.5f),
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_PLACE_UPPER_BUCKET)),
                     new SleepAction(1),

                     moveToSample1Place,

                     arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                     new SleepAction(.5),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),

                     new SleepAction(.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB),

                     new ParallelAction(
                             new SequentialAction(
                                 new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                                 new SleepAction(1.5f),
                                 new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB))
                             ),
                             moveToPreSample2Pickup
                     ),

                     new SleepAction(.5f),
                     moveToSample2Pickup,


                     arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                     new SleepAction(0.25),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                     new SleepAction(0.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                     moveToPreSample2Place,


                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                    new SleepAction(0.5f),
                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_PLACE_UPPER_BUCKET)),
                    new SleepAction(1),

                    moveToSample2Place,

                     arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                     new SleepAction(.5),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),

                    new SleepAction(.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB),

                     new ParallelAction(
                             new SequentialAction(
                                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                                     new SleepAction(1.5f),
                                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB))
                             ),
                             moveToPreSample3Pickup
                     ),

                     new SleepAction(.5f),
                     moveToSample3Pickup,

                    arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                    new SleepAction(0.25),
                    arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                    new SleepAction(0.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                     moveToPreSample3Place,


                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                     new SleepAction(0.5f),
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_PLACE_UPPER_BUCKET)),
                     new SleepAction(1),

                     moveToSample3Place,

                     arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                     new SleepAction(.5),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),

                     new SleepAction(.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB_FOURTH),

                     new ParallelAction(
                             new SequentialAction(
                                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                                     new SleepAction(1.5f),
                                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB))
                             ),
                             moveToPreSample4Pickup
                     ),

                     new SleepAction(2),

                     new InstantAction(() -> shouldUpdatePIDMotors=false)
                )
        );
    }
}