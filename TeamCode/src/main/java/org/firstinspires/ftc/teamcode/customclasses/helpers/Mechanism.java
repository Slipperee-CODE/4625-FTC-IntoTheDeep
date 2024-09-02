package org.firstinspires.ftc.teamcode.customclasses.helpers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Mechanism implements MechanismInterface {

    protected CustomGamepad gamepad;

    @Override
    public abstract void update();

    @Override
    public void update(Telemetry telemetry){
        update();
    }
}
