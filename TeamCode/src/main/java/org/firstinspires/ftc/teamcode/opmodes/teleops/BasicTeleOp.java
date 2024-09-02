package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customclasses.helpers.AprilTagAlign;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomOpMode;
import org.firstinspires.ftc.teamcode.customclasses.helpers.MultiProcessorWebcam;
import org.firstinspires.ftc.teamcode.customclasses.helpers.PoseStorage;

@TeleOp(name="BasicTeleOp", group="TeleOp")
public class BasicTeleOp extends CustomOpMode
{
    public static final double DPAD_SPEED = 0.5;
    CustomGamepad gamepad1;
    private AprilTagAlign tagAlign;
    private MultiProcessorWebcam multiProcessorWebcam;

    public void init(){
        super.init();
        robotDrivetrain.setSpeedConstant(0.85);
        roadrunnerDrivetrain.setPoseEstimate(PoseStorage.currentPose);
        gamepad1 = new CustomGamepad(this,1);
        tagAlign = new AprilTagAlign(hardwareMap,telemetry,gamepad1,robotDrivetrain);

        multiProcessorWebcam = new MultiProcessorWebcam(hardwareMap);
        multiProcessorWebcam.setActiveProcessor(MultiProcessorWebcam.Processor.APRIL_TAG);
        multiProcessorWebcam.setExposure(2);
        multiProcessorWebcam.setGain(0);
    }

    public void start() {

    }

    public void loop() {
        gamepad1.update();

        if (gamepad1.guideDown) {
            robotDrivetrain.switchDirection();
        }
        if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
            double horizontal = 0;
            double vert = 0;
            if (gamepad1.dpad_left) horizontal += DPAD_SPEED;
            if (gamepad1.dpad_right) horizontal -= DPAD_SPEED;
            if (gamepad1.dpad_up) vert -= DPAD_SPEED;
            if (gamepad1.dpad_down) vert += DPAD_SPEED;
            robotDrivetrain.emulateController(vert,horizontal,0);
        } else {
            robotDrivetrain.emulateController(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * 0.5);
        }

        if (gamepad1.gamepad.right_trigger > 0.3 ) {
            tagAlign.setState(MechanismState.NEAREST);
        } else {
            tagAlign.setState(MechanismState.OFF);
        }
        multiProcessorWebcam.update();

        telemetry.update();
    }
}