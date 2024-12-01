package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

public abstract class RRCustomOpMode extends OpMode {
    protected RobotDrivetrain robotDrivetrain;
    protected MecanumDrive roadrunnerDrivetrain;
    protected List<Action> runningActions = new ArrayList<>();
    private FtcDashboard dash = FtcDashboard.getInstance();

    @Override //Overrides OpMode's init()
    public void init(){
        robotDrivetrain = new RobotDrivetrain(hardwareMap);
        roadrunnerDrivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    @Override //Overrides OpMode's init_loop()
    public void init_loop(){}

    @Override //Overrides OpMode's start()
    public void start(){}

    @Override //Overrides OpMode's loop()
    public void loop(){

    }

    @Override //Overrides OpMode's stop()
    public void stop(){
        robotDrivetrain.setAllMotorPowers(0.0);
    }

    public void runActions(){
        if (runningActions.isEmpty()) return;
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions){
            action.preview(packet.fieldOverlay());
            if (action.run(packet)){
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
