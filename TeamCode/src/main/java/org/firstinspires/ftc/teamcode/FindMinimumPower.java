package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "FindMinimumPower")

public class FindMinimumPower extends Coyote {
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    double minimumPower = 0;
    double power = 0;
    @Override
    public void runOpMode() {

        initialize(new Pose(0,0,0));
        waitForStart();
        while(opModeIsActive()) {
            while(minimumPower==0) {
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
                sleep(300);
                power += .01;
                if(((Math.pow(velocityPose.x*velocityPose.x+velocityPose.y*velocityPose.y,.5)) >1.2)){
                    minimumPower=power;
                }
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            packet.put("MinimumPower",minimumPower);
            dashboard.sendTelemetryPacket(packet);
        }
    }}

