package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRCustomOpMode;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRClaw;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRStateHandler;

@Config
@TeleOp(name="RRBasicTeleOp", group="TeleOp")
public class RRBasicTeleOp extends RRCustomOpMode
{ 
    public static final double DPAD_SPEED = 0.25;
    CustomGamepad gamepad1;
    CustomGamepad gamepad2;
    private RRArm arm;
    private RRStateHandler stateHandler;

    @Override
    public void init(){
        super.init();
        robotDrivetrain.setSpeedConstant(.75);
        //Load roadrunner pose2D from auto instead of doing the line below
        roadrunnerDrivetrain.setPoseEstimate(new Pose2d(0, 0, 0)); // temp line


        gamepad1 = new CustomGamepad(this,1);
        gamepad2 = new CustomGamepad(this, 2);
        arm = new RRArm(hardwareMap, gamepad2, telemetry);
        stateHandler = new RRStateHandler(roadrunnerDrivetrain, gamepad1);
    }

    @Override
    public void init_loop(){
        arm.claw.initUpdateForGrab();
        gamepad2.update();
    }

    @Override
    public void start(){
        runningActions.add(arm.queueUpdateActions());
        //runningActions.add(arm.claw.setClawState(RRClaw.ClawPos.POST_GRAB));
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

        } else if (gamepad1.b) {
            robotDrivetrain.emulateController(0,0,0.25f);
        }
        else if (gamepad1.x) {
            robotDrivetrain.emulateController(0,0,-0.25f);
        }
        else {
            robotDrivetrain.emulateController(Math.pow(gamepad1.left_stick_y, 3), Math.pow(-gamepad1.left_stick_x, 3), Math.pow(gamepad1.right_stick_x * 1.0f,3));
        }

        runningActions = arm.queueActions(runningActions, telemetry);
        //runningActions = stateHandler.queueActions(runningActions, telemetry);
        runActions();
    }
}