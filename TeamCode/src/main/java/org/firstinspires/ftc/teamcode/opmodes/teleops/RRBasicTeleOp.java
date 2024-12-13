package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRCustomOpMode;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRClaw;

@Config
@TeleOp(name="RRBasicTeleOp", group="TeleOp")
public class RRBasicTeleOp extends RRCustomOpMode
{ 
    public static final double DPAD_SPEED = 0.25;
    CustomGamepad gamepad1;
    CustomGamepad gamepad2;
    private RRArm arm;

    @Override
    public void init(){
        super.init();
        robotDrivetrain.setSpeedConstant(.65);
        gamepad1 = new CustomGamepad(this,1);
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2);
    }

    @Override
    public void init_loop(){
        arm.claw.initUpdateForGrab();
        gamepad2.update();
    }

    @Override
    public void start(){
        runningActions.add(arm.queueUpdateActions());
        runningActions.add(arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB));
    }

    @Override
    public void loop() {
        gamepad1.update();
        gamepad2.update();

        if (gamepad1.aDown) {
            robotDrivetrain.switchDirection();
        }
        if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
            double horizontal = 0;
            double vert = 0;
            if (gamepad1.dpad_left) horizontal += DPAD_SPEED*2;
            if (gamepad1.dpad_right) horizontal -= DPAD_SPEED*2;
            if (gamepad1.dpad_up) vert -= DPAD_SPEED;
            if (gamepad1.dpad_down) vert += DPAD_SPEED;
            robotDrivetrain.emulateController(vert,horizontal,0);
        } else {
            robotDrivetrain.emulateController(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * 0.85);
        }

        runningActions = arm.queueActions(runningActions, telemetry);
        runActions();
    }
}