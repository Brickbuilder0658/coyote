package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.lang.Math;
/**
 * Variable for tracking position. Stores x,y and the robots angle.
 */
class Pose {
    Pose(double x, double y, double a) {
        this.x = x;
        this.y = y;
        angle = a;
    }

    public double x;
    public double y;

    public double angle;

    public String toString() {
        return "Pose(X = " + x + ", Y = " + y + ", Angle in Radians = " + angle + ", Angle in Degrees = " + Math.toDegrees(angle) + ")";
    }
}
/**
 * COYOTE - Control Of Your Own Trajectory Estimation
 *
 * @author Aidan
 *
 */
public class Coyote extends LinearOpMode {
    double lastTime;
    double oldTimeDecel;
    double currentTime;
    double oldTimeDriveTo;

    double timeStopped = 0;
    double timeStoppedDecel = 0;
    /**
     * Distance away from the final position where the robot will start to decelerate.
     */
    double startDecel = 30; // Made this up, could be changed. Tells the robot how far away too start decelerating.
    /**
     * Distance away from the final position where the robot is okay with stopping.
     */
    double stopDecel = .5; // Made this up aswell, could be changed. tells the robot how close it can be to a point before considering it as being there.
    /**
     * Pose that tracks your robots position.
     */
    static Pose fieldPose = new Pose(0, 0, 0);
    /**
     * Pose that tracks your robots velocity.
     */
    Pose velocityPose = new Pose(0, 0, 0);
    double zeroAngle = 0;

    double ratio = 0; // Find through the use of TestAngular
    double ticksPerRadian = ratio * (360 / (Math.PI * 2));
    /**
     * Ins per Tick for your robots encoders.
     */
    public double inPerTick = 0; // Determine through the FindInPerTick OpMode.

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    public IMU imu;
    public DcMotorEx deadPerp;

    public DcMotorEx deadLeft;

    public DcMotorEx deadRight;
    double previousAngle = 0;

    double spinCounter = 0;

    double maxVelocity = 0; // Determine through FindMaxVelocity OpMode.
    /**
     * The minimum power for your robot to move.
     */
    double minimumPower = 0; // Determine through FindMinimumPower OpMode. Do this at full battery.
    private Pose lastPose;

    /**
     * Updates the fieldPose variable.,
     */
    void asyncPositionCorrector() {
        if (lastPose == null) {
            lastPose = getCurrentPose();
            return;
        }
        Pose current = getCurrentPose();
        currentTime = System.currentTimeMillis() / 1000.0;
        double achange = current.angle - lastPose.angle;
        double xchange = current.x - lastPose.x;
        double ychange = current.y - lastPose.y;
        if (achange > Math.PI) {
            achange -= Math.PI * 2;
        } else if (achange < -Math.PI) {
            achange += Math.PI * 2;

        }
        double correctedx = xchange - achange * ticksPerRadian * inPerTick;
        double rotA = current.angle / 2 + lastPose.angle / 2;
        double rotX = correctedx * Math.cos(rotA) - ychange * Math.sin(rotA);
        double rotY = correctedx * Math.sin(rotA) + ychange * Math.cos(rotA);
        lastPose = current;
        fieldPose.x += rotX;
        fieldPose.y += rotY;
        fieldPose.angle = current.angle;
        velocityPose.x = rotX / ((currentTime - lastTime));
        velocityPose.y = rotY / ((currentTime - lastTime));
        velocityPose.angle = achange / (currentTime - lastTime);
        lastTime = currentTime;
    }

    Pose getCurrentPose() {
        double y = (deadLeft.getCurrentPosition() * inPerTick - deadRight.getCurrentPosition() * inPerTick) / 2;
        return new Pose(deadPerp.getCurrentPosition() * inPerTick, y, (imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle - zeroAngle));
    }
    /**
     * Sets the desired motor to a certain power or a power closer to the motors current power to accelerate slower and prevent slippage.
     * @param motor the motor to set the power of.
     * @param targetPower the power to try and get the motor to.
     */

    public void setPower(DcMotor motor, double targetPower) {
        double currentPower = motor.getPower();
        double velocityVector = Math.pow((Math.pow(velocityPose.x,2)+Math.pow(velocityPose.y,2)),.5);
        //create state machine just in here to track how long your trying to speed up or go abckwards or whatever and to track time youve been in that
        if (Math.abs(targetPower) - .1 > Math.abs(currentPower)) {

            double powerChange = 0.0125 * Math.signum(targetPower); // Adjust power based on the sign of targetPower

            double newPower = currentPower + powerChange;

            // Make sure newPower stays within the valid range of -1.0 to 1.0
            newPower = Math.max(-1.0, Math.min(1.0, newPower));


            motor.setPower(newPower);
        } else {
            if (Math.abs(targetPower) > 0 && Math.abs(targetPower) < 0.2) {
                targetPower = Math.signum(targetPower) * .2;
            }
            motor.setPower(targetPower);
        }
    }
    /**
     * Starts sending the robot in the direction of the target.
     * @param target Position for the robot to go to.
     * @param slowDown Whether the robot should slowdown.
     * @return whether the robot has reached its destination
     */
    public boolean driveToPointAsync(Pose target, boolean slowDown) {

        Pose cur = fieldPose; // our current poe
        Pose diff = new Pose(target.x - cur.x, target.y - cur.y, wrap((target.angle) - (cur.angle))); // difference in points
        if(Math.pow(diff.x*diff.x+diff.y*diff.y,.5)>6&&Math.pow(velocityPose.x*velocityPose.x+velocityPose.y*velocityPose.y,.5)<1){
            timeStopped += System.currentTimeMillis()-oldTimeDriveTo;
        }
        else{
            timeStopped=0;
        }
        oldTimeDriveTo=System.currentTimeMillis();
        // diff is difference in position and cur is current position
        // uses angles to find rotated X and Y
        double rotX = diff.x * Math.cos(-cur.angle) - diff.y * Math.sin(-cur.angle);
        double rotY = diff.x * Math.sin(-cur.angle) + diff.y * Math.cos(-cur.angle);
        double denom = Math.max(Math.abs(rotX), Math.abs(rotY));
        if (denom != 0) {
            rotX = rotX / denom;
            rotY = rotY / denom;
        }
        double angleFactor = diff.angle;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(angleFactor), 1);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double multiplier = deceleration(slowDown, diff.x, diff.y, diff.angle);
        double frontLeftPower = ((rotY + rotX - angleFactor) / denominator) * multiplier;
        double backLeftPower = ((rotY - rotX - angleFactor) / denominator) * multiplier;
        double frontRightPower = ((rotY - rotX + angleFactor) / denominator) * multiplier;
        double backRightPower = ((rotY + rotX + angleFactor) / denominator) * multiplier;

//        packet.put("frontleftPower",frontLeftPower);
//        packet.put("fronrightPower",frontRightPower);
//        packet.put("backrightpower",backRightPower);
//        packet.put("backleftPower",backLeftPower);
//        if(Math.abs(target.angle)==Math.PI/2){NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
//            frontLeftPower*=-1;
//            frontRightPower*=-1;
//            backLeftPower*=-1;
//            backRightPower*=-1;
//        }
        if(timeStopped>5000){
            frontRight.setPower(-frontRightPower);
            frontLeft.setPower(-frontLeftPower);
            backRight.setPower(-backRightPower);
            backLeft.setPower(-backLeftPower);
            sleep(1500);
            timeStopped=0;
            return false;
        }
        setPower(frontRight, frontRightPower);
        setPower(frontLeft, frontLeftPower);
        setPower(backRight, backRightPower);
        setPower(backLeft, backLeftPower);
        if (multiplier == 0) {
            return true;
        }
        return false;
    }
    /**
     * Moves the robot to its target destination.
     * @param target Position for the robot to go to.
     * @param slowDown Whether the robot should slowdown.
     */
    void driveToPoint(Pose target, boolean slowDown) {
        boolean done = false;
        asyncPositionCorrector();
        while(!done && opModeIsActive()){
            asyncPositionCorrector();
            done = driveToPointAsync(target, slowDown);
        }
    }
    protected void motorsStop() {
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    protected double deceleration(boolean slow, double rotX, double rotY, double angleDiff) {
        boolean slowDown = slow;

        double angleConstant = .1 / (10 * (Math.PI * 2) / 360);
        double d = Math.sqrt(((rotX * rotX) + (rotY * rotY)));// + Math.abs(angleDiff * angleConstant); Accounting for angle with distance
        double velocityVector = Math.pow((Math.pow(velocityPose.x,2)+Math.pow(velocityPose.y,2)),.5);
        if(velocityVector < 1){
            timeStoppedDecel+= currentTime - oldTimeDecel;
        }
        else{
            timeStoppedDecel=0;
        }
        if(timeStoppedDecel>5000){
            timeStoppedDecel=5000;
        }
        oldTimeDecel = currentTime;
        if ((d <= stopDecel)) {
            return 0;
        }
        if (slowDown) {
            if (d < startDecel) {
//                double powerLinear = ((.7 / (startDecel - stopDecel)) * (d - stopDecel) + .3) - 1 * (Math.sqrt((velocityPose.x * velocityPose.x) + (velocityPose.y * velocityPose.y)) / maxVelocity);
//                return powerLinear;
//            }
                double powerLinear = (((1 - minimumPower) / (startDecel - stopDecel)) * (d - stopDecel) + minimumPower);
                //will want to change velocityRatio to a real algorithm
                double velocityRatio = (Math.sqrt((velocityPose.x * velocityPose.x) + (velocityPose.y * velocityPose.y)) / maxVelocity);
                if(((powerLinear - velocityRatio)*((5000+timeStoppedDecel)/5000))<1){
                    return((powerLinear - velocityRatio)*((5000+timeStoppedDecel)/5000));
                }
                return(1);
            }
//           else if (targetspeed - speed > 0.02) {
//                //Motors would get faster
//                return 1;
            //  }
//            else {
//                //Speed is normal
//                return 1;
//            }
            else if (d < .5) {
                return 0;

            } else if (d < 6) {
                return .5;
            }

            return 1;
        }
        return 1;
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while (Math.abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }

    public double positiveWrap(double theta) {
        double newTheta = theta;
        while (newTheta > Math.PI * 2) {
            newTheta -= Math.PI * 2;
        }
        while (newTheta < 0) {
            newTheta += Math.PI * 2;
        }
        return newTheta;
    }

    /**
     * Initiliazes the robot.
     * @param inputPose Starting position for the robot
     */
    protected void initialize(Pose inputPose) {
        // Change each of these to match your hardware map
        frontLeft = hardwareMap.get(DcMotorEx.class, ""); // Front Left Motor
        backLeft = hardwareMap.get(DcMotorEx.class, ""); // Back left Motor
        frontRight = hardwareMap.get(DcMotorEx.class, ""); // Front Right Motor
        backRight = hardwareMap.get(DcMotorEx.class, ""); // Back Right Motor
        deadLeft = hardwareMap.get(DcMotorEx.class, ""); // Left Parallel Odometry wheel
        deadRight = hardwareMap.get(DcMotorEx.class, ""); // Right Parallel Odometry wheel
        deadPerp = hardwareMap.get(DcMotorEx.class, ""); // Perpendiculur Odometry wheel


        deadLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        imu = hardwareMap.get(IMU.class, "imu");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();

        fieldPose = inputPose;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        sleep(500);
        // if robot takes zeroangle too much it breaks.
        zeroAngle = (imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle) - inputPose.angle;
    }

    /**
     * Rotates the robot to the designated angle.
     * @param angle target angle for the robot
     */
    void turnRobot(double angle) {
        double difAngle = wrap(positiveWrap(angle) - positiveWrap(fieldPose.angle));
        double directionalSpeed;
        while (opModeIsActive() && !(Math.abs(difAngle) < Math.toRadians(2))) {
            asyncPositionCorrector();
            difAngle = wrap(positiveWrap(angle) - positiveWrap(fieldPose.angle));
            directionalSpeed = -Math.signum(difAngle) * 0.5;
            if (Math.abs(difAngle) < 0.08 * Math.PI) {
                directionalSpeed *= 0.25;
            } else if (Math.abs(difAngle) < 0.15 * Math.PI) {
                directionalSpeed *= 0.4;
            }


            setPower(backLeft, directionalSpeed);
            setPower(backRight, -directionalSpeed);
            setPower(frontLeft, directionalSpeed);
            setPower(frontRight, -directionalSpeed);

        }

        setPower(backLeft, 0);
        setPower(backRight, 0);
        setPower(frontLeft, 0);
        setPower(frontRight, 0);
    }
    protected void imuAngle () {
        telemetry.addData("IMU Angle", getCurrentPose().angle);
        telemetry.update();
    }


    protected void Forward(){
        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        backLeft.setPower(.5);
        backRight.setPower(.5);
    }


    // comment #2
    @Override
    public void runOpMode() {
    }

}