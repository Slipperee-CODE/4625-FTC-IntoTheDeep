package org.firstinspires.ftc.teamcode.customclasses.mechanisms;

import android.content.ActivityNotFoundException;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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

    public class Activate implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            packet.addLine("Activated ExampleMechanism");
            return false;
        }
    }
    public Action activate(){
        return new Activate();
    }

    public class Deactivate implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            packet.addLine("Deactivated ExampleMechanism");
            return false;
        }
    }
    public Action deactivate(){
        return new Deactivate();
    }
}
