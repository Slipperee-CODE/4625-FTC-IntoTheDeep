package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.customclasses.helpers.CustomGamepad;
import org.firstinspires.ftc.teamcode.customclasses.helpers.Mechanism;

public class ExampleMechanism extends Mechanism {

    public ExampleMechanism(CustomGamepad gamepad){
        this.gamepad = gamepad;
    }

    @Override
    public void update() {

    }

    @Override
    public void update(Telemetry telemetry) {

    }

    public void activate(){

    }

    public class Activate implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            packet.addLine("Activated ExampleMechanism");
            activate();
            return false; //Stops the method from looping
        }
    }
    public Action activateAction(){ return new Activate(); }

    public void deactivate(){

    }

    public class Deactivate implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            packet.addLine("Deactivated ExampleMechanism");
            deactivate();
            return false; //Stops the method from looping
        }
    }
    public Action deactivateAction(){
        return new Deactivate();
    }
}
