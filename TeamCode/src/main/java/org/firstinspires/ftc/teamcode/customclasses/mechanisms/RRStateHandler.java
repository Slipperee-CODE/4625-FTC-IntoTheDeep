package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.RRMechanism;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

public class RRStateHandler extends RRMechanism {

    private Pose2d firstEndpoint;
    private Pose2d secondEndpoint;

    private MecanumDrive roadrunnerDrivetrain;

    public RRStateHandler(MecanumDrive roadrunnerDrivetrain, CustomGamepad gamepad){
        this.roadrunnerDrivetrain = roadrunnerDrivetrain;
        this.gamepad = gamepad;
    }

    public List<Action> queueActions(List<Action> runningActions, Telemetry telemetry) {
        if (gamepad.yDown) {
            if (gamepad.yToggle) {
                firstEndpoint = roadrunnerDrivetrain.pose;
            } else {
                secondEndpoint = roadrunnerDrivetrain.pose;
            }
        }

        if (gamepad.bDown) {
            if (gamepad.bToggle) {
                Action goToFirstEndPoint = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                        .strafeToLinearHeading(firstEndpoint.position, firstEndpoint.heading)
                        .build();
                runningActions.add(
                        goToFirstEndPoint
                );
            } else {
                Action goToSecondEndPoint = roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                        .strafeToLinearHeading(secondEndpoint.position, secondEndpoint.heading)
                        .build();
                runningActions.add(
                        goToSecondEndPoint
                );
            }
        }
        return runningActions;
    }

    @Override
    public void queueActions(Telemetry telemetry) {

    }

    @Override
    public Action queueUpdateActions() {
        return null;
    }
}
