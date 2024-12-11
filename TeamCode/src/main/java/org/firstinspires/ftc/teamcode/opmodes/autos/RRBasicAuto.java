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

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(9, -64, -Math.PI/2));

        moveToFirstSpecimenPickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYConstantHeading(-38)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(34, -40, -Math.PI/2), 0)
                .splineToLinearHeading(new Pose2d(42, -12, -Math.PI/2), 0)
                .lineToX(50.5)
                .waitSeconds(3)
                .setTangent(Math.PI/2)
                .lineToY(-52)
                .splineToLinearHeading(new Pose2d(60, 0, -Math.PI/2), 0)
                /*
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(61, -6, -Math.PI/2), 0)
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(37, -46, -Math.PI/2), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(9, -34, -Math.PI/2), Math.PI)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(37, -46, -Math.PI/2), -Math.PI/2)
                */
                .build();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        gamepad2.update();
        //arm.claw.initUpdateForGrab();
    }

    @Override
    protected void startAfterWait() {
        Actions.runBlocking(
                new ParallelAction(
                        arm.queueUpdateActions(),
                        new SequentialAction(
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET)),
                                new SleepAction(1),
                                moveToFirstSpecimenPickup,
                                new InstantAction(() -> arm.deactivatePIDMotors()) //SUPER IMPORTANT LINE BECAUSE IT PREVENTS AN INFINITE LOOP WHEN STOPPED

                                //arm.claw.setClawState(RRClaw.ClawPos.RESET)
                                //new InstantAction(() -> arm.setArmState(RRArm.ArmState.DEFAULT)),
                                //moveToFirstSamplePickup,
                                //arm.setupForSampleGrab(0.5f)
                        )
                )
        );
    }

    @Override
    public void stop(){
        arm.deactivatePIDMotors();
    }
}

