package org.firstinspires.ftc.teamcode.customclasses.helpers;

public abstract class WaitingAuto extends CustomOpMode {
    protected CustomGamepad gamepadOne;

    private final Clock __delayTimer = new Clock();
    private double time_to_start = 0.0;
    private boolean waiting = true;
    public void init() {
        super.init();
        gamepadOne = new CustomGamepad(this,1);
    }

    public void init_loop() {
        gamepadOne.update();

        if (gamepadOne.yDown) {
            time_to_start = 0.0;
        }
        time_to_start += gamepadOne.left_stick_y * 0.001;
        telemetry.addData("Time To Start: ",time_to_start);
        telemetry.addLine("Left Joystick to control");
        telemetry.addLine("Y to Reset to 0");
    }

    public final void start() {
        __delayTimer.reset();
        startBeforeWait();
    }

    protected void startBeforeWait() {}
    protected void startAfterWait() {}

    @Override
    public final void loop() {
        if (waiting){
            waiting = __delayTimer.getTimeSeconds() < time_to_start;
            if (!waiting) startAfterWait();
            return;
        }
        update();
    }

    protected void update() {};

    @Override
    public final void stop(){

    }
}

