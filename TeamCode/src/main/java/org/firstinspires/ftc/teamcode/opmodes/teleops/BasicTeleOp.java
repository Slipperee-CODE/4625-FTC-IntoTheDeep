package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customclasses.helpers.AprilTagAlign;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomOpMode;
import org.firstinspires.ftc.teamcode.customclasses.helpers.MultiProcessorWebcam;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.Arm;

@Config
@TeleOp(name="BasicTeleOp", group="TeleOp")
public class BasicTeleOp extends CustomOpMode
{
    public static final double DPAD_SPEED = 0.5;
    CustomGamepad gamepad1;
    CustomGamepad gamepad2;
    private Arm arm;

    @Override
    public void init(){
        super.init();
        robotDrivetrain.setSpeedConstant(.9);
        gamepad1 = new CustomGamepad(this,1);
        gamepad2 = new CustomGamepad(this, 2);
        arm = new Arm(hardwareMap, gamepad2);
    }

    @Override
    public void init_loop(){
        arm.claw.initUpdateForGrab();
        gamepad2.update();
    }

    @Override
    public void loop() {
        gamepad1.update();
        gamepad2.update();

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
            robotDrivetrain.emulateController(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * 0.85);
        }

        arm.updateWithBoolean(telemetry);
        telemetry.update();
    }
}