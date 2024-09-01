package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomGamepad {
    public final Gamepad gamepad;
    public boolean x,y,a,b,dpad_up,dpad_down,dpad_left,dpad_right,xDown,yDown,aDown,bDown,upDown,downDown,leftDown,rightDown;
    private boolean px,py,pa,pb,pup,pdown,pleft,pright;
    public boolean xToggle,yToggle,aToggle,bToggle,upToggle,downToggle,leftToggle,rightToggle;
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
    public boolean guideDown,guide; // the big center button on the controller
    private boolean pguide;

    public void update() {
        //Rebinds the gamepad's variables to our own use

        x     = gamepad.x;
        y     = gamepad.y;
        a     = gamepad.a;
        b     = gamepad.b;
        guide = gamepad.guide;
        dpad_up = gamepad.dpad_up;
        dpad_down = gamepad.dpad_down;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;

        xDown = x && !px;
        yDown = y && !py;
        aDown = a && !pa;
        bDown = b && !pb;
        upDown   = dpad_up && !pup;
        downDown  = dpad_down && !pdown;
        leftDown  = dpad_left && !pleft;
        rightDown = dpad_right && !pright;
        guideDown = guide && !pguide;

        px = x;
        py = y;
        pa = a;
        pb = b;
        pup = dpad_up;
        pdown = dpad_down;
        pleft = dpad_left;
        pright = dpad_right;
        pguide = guide;

        xToggle ^= xDown;
        yToggle ^= yDown;
        aToggle ^= aDown;
        bToggle ^= bDown;
        upToggle ^= upDown;
        downToggle ^= downDown;
        leftToggle ^= leftDown;
        rightToggle ^= rightDown;
    }

    public CustomGamepad(OpMode opMode, int gamepadNum){

        switch (gamepadNum){
            case 1:
                gamepad = opMode.gamepad1;
                break;

            case 2:
                gamepad = opMode.gamepad2;
                break;

            default:
                opMode.telemetry.addLine("WARNING: INVALID CUSTOMGAMEPAD NUMBER");
                opMode.telemetry.update();
                try{
                    wait(0L,Integer.MAX_VALUE);
                }
                catch (Exception ignored) {}
                gamepad = opMode.gamepad1;
        }
    }
}
