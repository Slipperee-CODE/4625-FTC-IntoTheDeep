package org.firstinspires.ftc.teamcode.customclasses.helpers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagAlign extends Mechanism {
    private final RobotDrivetrain robot;
    private final float STARTING_ERROR_BETWEEN_TAGS = 10; //in inches
    private final Telemetry telemetry;
    private int targetID = 0;
    private boolean isAligned;
    private boolean isAlignedPerfectly;
    private double intRot = 0;
    private double intStr = 0;
    private double intFor = 0;

    public AprilTagAlign(HardwareMap hardwareMap, Telemetry telemetry, CustomGamepad gamepad, RobotDrivetrain robot)
    {
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public void setTargetID(int num) {
        this.targetID = num;
        intRot = 0;
        intStr = 0;
        targetID = Math.max(Math.min(targetID,5),0); //CLIPPING to the range 0,5
    }
    public int getTargetID() { return targetID; }


    public void update()
    {
        List<AprilTagDetection> detectedTags = VisibleTagsStorage.stored;
        if (detectedTags == null) {
            telemetry.addLine("No Tags Detected");
            return;
        }

        switch (state) {
            case OFF:
                state = MechanismState.IDLE;
                break;
            case ON:
                if (gamepad != null){
                    if (gamepad.leftDown) {
                        targetID--;
                        targetID = Math.min(Math.max(targetID, 0), 6); //CLIPPING bDPos to the range 0,5
                    } else if (gamepad.rightDown) {
                        targetID++;
                        targetID = Math.min(Math.max(targetID, 0), 6); //CLIPPING bDPos to the range 0,5
                    }
                    telemetry.addData("Target ID:", targetID);

                }
                if (detectedTags.size() == 0) {
                    robot.stop();
                    telemetry.addLine("No Tags Detected");
                } else {
                    navigateToAprilTag(detectedTags);
                }
                break;
            case NEAREST:
                if (detectedTags.size() != 0) {
                    goToAprilTagNearest(detectedTags);
                } else {
                    robot.stop();
                }
                break;
            case IDLE:
                //WAITING FOR NEXT STATE
                break;

            default:
                state = MechanismState.IDLE;
        }
    }
    public void goToAprilTagNearest(List<AprilTagDetection> currentTags)
    {
        //HOW TO TUNE GAINS
        // set all to zero except one and change the number until it works
        // save it somewhere else and then repeat for all gains


        final double TURN_GAIN = 0.6; // tuned to 0.3
        final double STRAFE_GAIN = 3; // tuned to 3.0
        final double FORWARD_GAIN = 1.4;
        final double MAX_TURN = 0.6;
        final double MAX_FORWARD = 0.35; // This should be determined by how fast the robot can move while maintaining a still image
        final double MAX_STRAFE = 0.5; // This should be determined by how fast the robot can move while maintaining a still image
        final double FORWARD_OFFSET = 0.257; // in meters.  Target distance between tag and bobot.

        AprilTagDetection toDriveTo = getStrongestDetection(currentTags,true);
        boolean targetAquired = true;
        //if we get here then the variable "toDriveto" should be our target to drive to
        double forwardError = FORWARD_OFFSET-toDriveTo.rawPose.z;
        telemetry.addData("Forward Error: ",forwardError);
        Orientation rot = Orientation.getOrientation(toDriveTo.rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS); // Maybe find a way to get the y rotation in radians without calculating all the rotation
        double forwardPower = FORWARD_GAIN * Math.tanh(forwardError);

        // + is right, - is left. also the greater for.firstAngle is, the less we wanna strafe
        double strafePower = targetAquired ? STRAFE_GAIN * toDriveTo.rawPose.x : 0.3 * (targetID - toDriveTo.id) / Math.max(Math.abs(rot.firstAngle)*5,1);
        double rotPower = TURN_GAIN * rot.firstAngle;
        //telemetry.addData("Prelim Rot Power: ", rotPower);
        //telemetry.addData("Strafe Power: ", strafePower);
        //telemetry.addData("Trying to Align to ", targetID);
        if (targetAquired && !isAlignedPerfectly && Math.abs(forwardError) < 0.2) {
            // this is when we are locked in and see the one we want and we not aligned

            if (rotPower * intRot < 0) {intRot = 0;}
            if (forwardPower * intFor < 0) {intFor = 0;}
            if (rotPower * intStr < 0) {intStr = 0;}

            intRot += 0.003 * rotPower;
            intRot = Range.clip(intRot,-.3,.3);
            rotPower += intRot;

            intStr += 0.02 * strafePower;

            intStr = Range.clip(intStr,-.2,.2);
            intFor += 0.01 * forwardPower;
            intFor = Range.clip(intFor,-.3,.3);
            //telemetry.addData("Integral Strafe: ",intStr);
            forwardPower += intFor;
            strafePower += intStr;
        } else {
            intRot = 0;
            intStr = 0;
            intFor = 0;
        }
        //telemetry.addData("Turn: ", Math.abs(rotPower) < 0.05);
        //telemetry.addData("Forward: ", Math.abs(forwardError) < 0.05);
        //telemetry.addData("Strafe: ", Math.abs(strafePower) < 0.1);


        robot.baseMoveRobot(Range.clip(strafePower,-MAX_STRAFE,MAX_STRAFE),
                Range.clip(forwardPower,-MAX_FORWARD,MAX_FORWARD),
                Range.clip(rotPower,-MAX_TURN,MAX_TURN));
        isAlignedPerfectly = (
                (toDriveTo.id == targetID || targetID <= 0) &&
                        Math.abs(forwardError) < 0.005 &&
                        Math.abs(strafePower) < 0.05 &&
                        Math.abs(rotPower) < 0.03
        );
        isAligned = (
                (toDriveTo.id == targetID || targetID <= 0) &&
                        Math.abs(forwardError) < 0.007 &&
                        Math.abs(strafePower) < 0.1 &&
                        Math.abs(rotPower) < 0.05
        );
    }
    public void navigateToAprilTag(List<AprilTagDetection> currentTags)
    {
        //HOW TO TUNE GAINS
        // set all to zero except one and change the number until it works
        // save it somewhere else and then repeat for all gains


        final double TURN_GAIN = 0.6; // tuned to 0.3
        final double STRAFE_GAIN = 3; // tuned to 3.0
        final double FORWARD_GAIN = 1.4;
        final double MAX_TURN = 0.6;
        final double MAX_FORWARD = 0.35; // This should be determined by how fast the robot can move while maintaining a still image
        final double MAX_STRAFE = 0.5; // This should be determined by how fast the robot can move while maintaining a still image
        final double FORWARD_OFFSET = 0.257; // in meters.  Target distance between tag and bobot.

        AprilTagDetection toDriveTo = getStrongestDetection(currentTags,true);
        boolean targetAquired = (toDriveTo.id == targetID || targetID <= 0);
        //if we get here then the variable "toDriveto" should be our target to drive to
        double forwardError = FORWARD_OFFSET-toDriveTo.rawPose.z;
        telemetry.addData("Forward Error: ",forwardError);
        Orientation rot = Orientation.getOrientation(toDriveTo.rawPose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS); // Maybe find a way to get the y rotation in radians without calculating all the rotation
        double forwardPower = FORWARD_GAIN * Math.tanh(forwardError);

        // + is right, - is left. also the greater for.firstAngle is, the less we wanna strafe
        double strafePower = targetAquired ? STRAFE_GAIN * toDriveTo.rawPose.x : 0.3 * (targetID - toDriveTo.id) / Math.max(Math.abs(rot.firstAngle)*5,1);
        double rotPower = TURN_GAIN * rot.firstAngle;
        //telemetry.addData("Prelim Rot Power: ", rotPower);
        //telemetry.addData("Strafe Power: ", strafePower);
        //telemetry.addData("Trying to Align to ", targetID);
        if (targetAquired && !isAlignedPerfectly && Math.abs(forwardError) < 0.2) {
            // this is when we are locked in and see the one we want and we not aligned

            if (rotPower * intRot < 0) {intRot = 0;}
            if (forwardPower * intFor < 0) {intFor = 0;}
            if (rotPower * intStr < 0) {intStr = 0;}

            intRot += 0.003 * rotPower;
            intRot = Range.clip(intRot,-.3,.3);
            rotPower += intRot;

            intStr += 0.02 * strafePower;

            intStr = Range.clip(intStr,-.2,.2);
            intFor += 0.01 * forwardPower;
            intFor = Range.clip(intFor,-.3,.3);
            //telemetry.addData("Integral Strafe: ",intStr);
            forwardPower += intFor;
            strafePower += intStr;
        } else {
            intRot = 0;
            intStr = 0;
            intFor = 0;
        }
        //telemetry.addData("Turn: ", Math.abs(rotPower) < 0.05);
        //telemetry.addData("Forward: ", Math.abs(forwardError) < 0.05);
        //telemetry.addData("Strafe: ", Math.abs(strafePower) < 0.1);


        robot.baseMoveRobot(Range.clip(strafePower,-MAX_STRAFE,MAX_STRAFE),
                Range.clip(forwardPower,-MAX_FORWARD,MAX_FORWARD),
                Range.clip(rotPower,-MAX_TURN,MAX_TURN));
        isAlignedPerfectly = (
                (toDriveTo.id == targetID || targetID <= 0) &&
                        Math.abs(forwardError) < 0.005 &&
                        Math.abs(strafePower) < 0.05 &&
                        Math.abs(rotPower) < 0.03
        );
        isAligned = (
                (toDriveTo.id == targetID || targetID <= 0) &&
                        Math.abs(forwardError) < 0.007 &&
                        Math.abs(strafePower) < 0.1 &&
                        Math.abs(rotPower) < 0.05
        );
    }
    public boolean isAligned() {
        return isAligned;
    }
    public boolean isAlignedPerfectly() {
        return isAlignedPerfectly;
    }

    // HELPER FUNCTIONS
    private double poseDistance(AprilTagPoseRaw pose) {
        return Math.sqrt(poseDistanceSqrd(pose));
    }
    private double poseDistanceSqrd(AprilTagPoseRaw pose) {
        return pose.x* pose.x + pose.z*pose.z;
    }
    private AprilTagDetection getStrongestDetection(List<AprilTagDetection> tags,boolean preferTarget) {
        if (tags == null || tags.size() == 0) return null;
        AprilTagDetection strongestDetection = null;
        double shortestDistanceSqrd = Double.POSITIVE_INFINITY;
        for (AprilTagDetection detection : tags) {
            double dist = poseDistanceSqrd(detection.rawPose);
            if (preferTarget && detection.id == targetID) return detection;
            if (dist < shortestDistanceSqrd) {
                shortestDistanceSqrd = dist;
                strongestDetection = detection;
            }
        }
        return strongestDetection;
    }

}