package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotDrivetrain {
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    private double speedConstant;
    private int direction = 1;

    public RobotDrivetrain(HardwareMap hwMap){
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void switchDirection(){
        setDirection(-direction);
    }

    public void setDirection(int direction) {
        assert (direction == 0 || direction == 1 || direction == -1);
        this.direction = direction;
    }

    /**
     * Will control robot as if they were from the robot controller
     * It will invert the left_y as going up on a controller will be negative
     * and down is positive.
     * @param left_y
     * @param left_x
     * @param right_x
     */
    public void emulateController(double left_y,double left_x,double right_x) {
        double y = left_y * -direction; // Forward-Backward and invert the y direction
        double x = left_x * direction; // Strafe
        double rx = right_x; // Rotate
        baseMoveRobot(x,y,rx);
    }

    public void baseMoveRobot(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x + rx) / denominator;
        double backLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        rightFront.setPower(frontRightPower * speedConstant);
        rightBack.setPower(backRightPower * speedConstant);
        leftFront.setPower(frontLeftPower * speedConstant);
        leftBack.setPower(backLeftPower * speedConstant);
    }

    public void stop() {
        setAllMotorPowers(0);
    }

    public void setAllMotorPowers(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    public void setSpeedConstant(double speedConstant){
        this.speedConstant = speedConstant;
    }
}