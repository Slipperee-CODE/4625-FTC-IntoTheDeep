package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class CustomOpMode extends OpMode {
    protected RobotDrivetrain robotDrivetrain;
    protected MecanumDrive roadrunnerDrivetrain;
    protected Clock timer = new Clock();

    @Override //Overrides OpMode's init()
    public void init(){
        robotDrivetrain = new RobotDrivetrain(hardwareMap);
        //roadrunnerDrivetrain = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    }

    @Override //Overrides OpMode's init_loop()
    public void init_loop(){}

    @Override //Overrides OpMode's start()
    public void start(){}

    @Override //Overrides OpMode's loop()
    public void loop(){}

    @Override //Overrides OpMode's stop()
    public void stop(){
        robotDrivetrain.setAllMotorPowers(0.0);
    }

    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
