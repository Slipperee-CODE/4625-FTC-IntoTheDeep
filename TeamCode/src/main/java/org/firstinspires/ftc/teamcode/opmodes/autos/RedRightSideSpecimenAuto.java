package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.customclasses.helpers.MultiProcessorWebcam;
import org.firstinspires.ftc.teamcode.customclasses.helpers.WaitingAuto;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.ExampleMechanism;

@Config
@Autonomous(name = "RedRightSideSpecimenAuto", group = "Autonomous")
public class RedRightSideSpecimenAuto extends WaitingAuto {
    private Arm arm;
    private Action mainTrajectory;

    @Override
    public void init() {
        super.init();
        arm = new Arm(hardwareMap);

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(-9, -64, Math.PI/2));

        mainTrajectory = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYConstantHeading(-52)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-48, -46, Math.PI/2), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, -46, Math.PI/2), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58, -46, 2*Math.PI/3), 0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4),0)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-24, 8, 0),0)
                .waitSeconds(1)
                .build();

        //Actions.runBlocking(exampleMechanism.activateAction());
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    protected void startAfterWait() {
        Actions.runBlocking(
                new SequentialAction(
                        arm.setArmStateAction(Arm.ArmState.UPPER_BAR, telemetry),
                        mainTrajectory
                        //mainTrajectory,
                        //arm.setArmStateAction(Arm.ArmState.UPPER_BAR, telemetry)
                )
        );
    }
}

