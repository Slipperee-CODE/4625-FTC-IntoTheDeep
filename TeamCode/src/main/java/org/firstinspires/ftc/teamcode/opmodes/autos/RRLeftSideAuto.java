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

    private Clock clock;
    private double startTime = 0.0;

    private boolean shouldPIDMotorsBeRunning = false;

    @Override
    public void init() {
        super.init();
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);
        clock = new Clock();


        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-38, -64, Math.PI/2));


        moveToPreSample1Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .setTangent(Math.PI/2)
                .lineToY(-55)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample1Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToSample2Pickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-48, -48), Math.PI/2)
                .build();


        moveToPreSample2Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample2Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();

        moveToSample3Pickup = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-58, -48), Math.PI/2)
                .build();


        moveToPreSample3Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.PI/4)
                .build();

        moveToSample3Place = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.PI/4)
                .build();
        
        park = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
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
        runActions();
        telemetry.update();
    }

    public class ManualArmPIDUpdate implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!shouldPIDMotorsBeRunning){
                arm.armExtender.closePivotPIDMotor.setRawPower(0);
                arm.armExtender.farPivotPIDMotor.setRawPower(0);
                arm.armPivoter.leftPivotPIDMotor.setRawPower(0);
                arm.armPivoter.rightPivotPIDMotor.setRawPower(0);
                return false;
            }
            arm.queueActions();
            return shouldPIDMotorsBeRunning;
        }
    }

    @Override
    protected void startAfterWait() {
        shouldPIDMotorsBeRunning = true;
        runningActions.add(
                new ParallelAction(
                        //new InstantAction(() -> startTime = clock.getTimeSeconds()),
                        new ManualArmPIDUpdate(),
                        new SequentialAction(
                                //moveToPreSample1Place,
                                new SleepAction(2),
                                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET))
                                //samplePlaceSequenceAction(moveToPreSample1Place, moveToSample1Place)
                                //samplePickupSequenceAction(moveToSample2Pickup)
                                //samplePlaceSequenceAction(moveToPreSample2Place, moveToSample2Place),
                                //samplePickupSequenceAction(moveToSample3Pickup),
                                //samplePlaceSequenceAction(moveToPreSample3Place, moveToSample3Place),
                                //park,
                        )
                        //new InstantAction(() -> telemetry.addData("Frame Time", clock.getTimeSeconds()-startTime))
                )
        );
    }

    @Override
    public void stop(){
        shouldPIDMotorsBeRunning = false;
        arm.deactivatePIDMotors();
    }

    private Action samplePlaceSequenceAction(Action enterTrajectory, Action placeTrajectory) {
        return new SequentialAction(
                enterTrajectory,
                new InstantAction(() -> robotDrivetrain.setAllMotorPowers(0)),
                new SleepAction(5),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.UPPER_BUCKET))
                /*
                new SleepAction(5),

                arm.preSampleDeposit(),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SPECIMEN_PLACE_UPPER_BUCKET)),

                new SleepAction(5),

                placeTrajectory,

                new SleepAction(5),

                arm.sampleDeposit(),

                new SleepAction(2),

                arm.setupForSampleGrab(1.0f),

                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_EXTENSION_REDUCTION_FOR_ARM_SAFETY)),

                new SleepAction(5)

                 */
        );
    }

    private Action samplePickupSequenceAction(Action exitTrajectory){
        return new SequentialAction(
                exitTrajectory,
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAMPLE_GRAB)),
                new SleepAction(2),
                arm.grabSample(),
                new SleepAction(2),
                new InstantAction(() -> arm.setArmState(RRArm.ArmState.AUTO_SAFE_DEFAULT))
        );
    }
}