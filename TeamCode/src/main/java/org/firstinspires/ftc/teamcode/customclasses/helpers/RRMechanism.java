package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public abstract class RRMechanism {

    protected CustomGamepad gamepad;
    protected List<Action> runningActions = null;

    public abstract void queueActions();

    public void queueActions(Telemetry telemetry){
        queueActions();
    }
}
