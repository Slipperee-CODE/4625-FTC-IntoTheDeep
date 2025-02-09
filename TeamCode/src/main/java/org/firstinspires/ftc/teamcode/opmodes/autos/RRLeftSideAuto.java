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

    private boolean shouldUpdatePIDMotors = true;

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
                .strafeToLinearHeading(new Vector2d(-47, -45), Math.PI/2)
                .build();


        moveToPreSample2Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-47,-45,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample2Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-50,-50,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToSample3Pickup = roadrunnerDrivetrain.actionBuilder(new Pose2d(-54,-54,Math.PI/4))
                .strafeToLinearHeading(new Vector2d(-53, -45), Math.PI/2)
                .build();


        moveToPreSample3Place = roadrunnerDrivetrain.actionBuilder(new Pose2d(-53,-45,Math.PI/2))
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
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                     new SleepAction(1),

                     moveToSample1Place,

                     arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                     new SleepAction(.25),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),

                     new SleepAction(.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                     new SleepAction(1.5f),
                     moveToSample2Pickup,
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB)),
                     new SleepAction(2),

                     arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                     new SleepAction(0.25),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                     new SleepAction(0.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                     moveToPreSample2Place,


                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                    new SleepAction(0.5f),
                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                    new SleepAction(1),

                    moveToSample2Place,

                     arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                     new SleepAction(.25),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),

                    new SleepAction(.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                    new SleepAction(1.5f),
                    moveToSample3Pickup, //Might be able to combine
                    new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB)), //this two lines in one parallel action
                    new SleepAction(2),

                    arm.claw.setClawState(RRClaw.ClawPos.SAMPLE_GRAB),
                    new SleepAction(0.25),
                    arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),
                    new SleepAction(0.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_DEPOSIT),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT)),
                     moveToPreSample3Place,


                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                     new SleepAction(0.5f),
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),
                     new SleepAction(1),

                     moveToSample3Place,

                     arm.claw.setClawState(RRClaw.ClawPos.RELEASE_SAMPLE),
                     new SleepAction(.25),
                     arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB),

                     new SleepAction(.25),

                     arm.claw.setClawState(RRClaw.ClawPos.PRE_SAMPLE_GRAB),

                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),
                     new SleepAction(2),

                     arm.claw.setClawState(RRClaw.ClawPos.RESET),
                     new InstantAction(() -> arm.setArmState(RRArm.ArmState.SAFE_DEFAULT)),

                     new SleepAction(1),

                     new InstantAction(() -> shouldUpdatePIDMotors=false)
                )
        );
    }
}