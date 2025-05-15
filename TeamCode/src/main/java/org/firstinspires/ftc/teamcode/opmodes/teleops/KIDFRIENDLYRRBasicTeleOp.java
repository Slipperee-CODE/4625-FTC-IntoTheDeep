package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRCustomOpMode;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRArm;
import org.firstinspires.ftc.teamcode.customclasses.mechanisms.RRStateHandler;

import java.util.List;

@Config
@TeleOp(name="KIDFRIENDLYRRBasicTeleOp", group="TeleOp")
public class KIDFRIENDLYRRBasicTeleOp extends RRCustomOpMode
{ 
    public static final double DPAD_SPEED = 0.25;
    CustomGamepad gamepad1;
    CustomGamepad gamepad2;
    private RRArm arm;

    @Override
    public void init(){
        super.init();
        robotDrivetrain.setSpeedConstant(.33);

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
            robotDrivetrain.emulateController(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x * 1.0f);
        }

        List<Action> armActions = arm.queueActions(runningActions, telemetry, false);

        ParallelAction armAndStateHandlerActions =
            new ParallelAction(
                armActions.toArray(new Action[0])
            );
        runningActions.add(armAndStateHandlerActions);

        roadrunnerDrivetrain.updatePoseEstimate();
        arm.queueActions(telemetry);
        runActions();
        telemetry.update();
    }
}