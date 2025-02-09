package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
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
    Action goToOtherEndpoint = null;

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

        if (gamepad.aDown) {
            if (gamepad.aToggle) {
                if (goToOtherEndpoint != null){
                    ((CancelableFollowTrajectoryAction) goToOtherEndpoint).cancelAbruptly();
                }
                goToOtherEndpoint = new CancelableFollowTrajectoryAction(
                        roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                        .strafeToLinearHeading(firstEndpoint.position, firstEndpoint.heading)
                        .build()
                );
                runningActions.add(
                        goToOtherEndpoint
                );
            } else {
                if (goToOtherEndpoint != null){
                    ((CancelableFollowTrajectoryAction) goToOtherEndpoint).cancelAbruptly();
                }
                goToOtherEndpoint = new CancelableFollowTrajectoryAction(
                        roadrunnerDrivetrain.actionBuilder(roadrunnerDrivetrain.pose)
                        .strafeToLinearHeading(secondEndpoint.position, secondEndpoint.heading)
                        .build()
                );
                runningActions.add(
                        goToOtherEndpoint
                );
            }
        }

        if (gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0){
            ((CancelableFollowTrajectoryAction) goToOtherEndpoint).cancelAbruptly();
        }

        return runningActions;
    }

    public class CancelableFollowTrajectoryAction implements Action {
        private Action action;
        private boolean cancelled = false;

        public CancelableFollowTrajectoryAction(Action action) {
            this.action = action;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (cancelled) {
                roadrunnerDrivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }

            return action.run(telemetryPacket);
        }

        public void cancelAbruptly() {
            cancelled = true;
        }
    }

    @Override
    public void queueActions(Telemetry telemetry) {

    }

    @Override
    public Action queueUpdateActions() {
        return null;
    }
}
