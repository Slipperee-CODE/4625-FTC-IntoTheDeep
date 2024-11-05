package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.customclasses.helpers.MultiProcessorWebcam;
import org.firstinspires.ftc.teamcode.customclasses.helpers.WaitingAuto;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.ExampleMechanism;

@Config
@Autonomous(name = "BasicAuto", group = "Autonomous")
public class BasicAuto extends WaitingAuto {
    private ExampleMechanism exampleMechanism;
    private Action trajectoryAction1;
    private Action trajectoryAction2;
    private Action trajectoryAction3;
    private Action trajectoryActionCloseOut;
    private Action chosenTrajectory;
    private int visionResult = 0;

    private MultiProcessorWebcam multiProcessorWebcam;

    @Override
    public void init() {
        super.init();
        exampleMechanism = new ExampleMechanism(gamepadOne);

        /*
        multiProcessorWebcam = new MultiProcessorWebcam(hardwareMap);
        multiProcessorWebcam.setActiveProcessor(MultiProcessorWebcam.Processor.CUSTOM);
        multiProcessorWebcam.setExposure(2);
        multiProcessorWebcam.setGain(0);
         */

        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(0,0,0));

        trajectoryAction1 = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYSplineHeading(10, Math.toRadians(0))
                .waitSeconds(2)
                .build();
        trajectoryAction2 = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYSplineHeading(10, Math.toRadians(0))
                .waitSeconds(2)
                .build();
        trajectoryAction3 = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYSplineHeading(10, Math.toRadians(0))
                .waitSeconds(2)
                .build();
        trajectoryActionCloseOut = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                .lineToYSplineHeading(0, Math.toRadians(0))
                .waitSeconds(2)
                .build();

        //Actions.runBlocking(exampleMechanism.activateAction());
    }

    @Override
    public void init_loop() {
        super.init_loop();
        //multiProcessorWebcam.update();
        //visionResult = multiProcessorWebcam.getResultFromCustomProcessor();
    }

    @Override
    protected void startBeforeWait() {
        switch (visionResult){
            case 1:
                chosenTrajectory = trajectoryAction1;
                break;
            case 2:
                chosenTrajectory = trajectoryAction2;
                break;
            case 3:
                chosenTrajectory = trajectoryAction3;
                break;
            default:
                chosenTrajectory = trajectoryAction1;
        }
    }

    @Override
    protected void startAfterWait() {
        Actions.runBlocking(
                new SequentialAction(
                        chosenTrajectory,
                        exampleMechanism.activateAction(),
                        trajectoryActionCloseOut,
                        exampleMechanism.deactivateAction()
                )
        );
    }
}
