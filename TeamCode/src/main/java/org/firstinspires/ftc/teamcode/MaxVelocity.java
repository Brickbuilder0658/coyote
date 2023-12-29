package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "FindMaxVelocity")

public class MaxVelocity extends Coyote {
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    double maxVelocity = 0;
    @Override
    public void runOpMode() {

        initialize(new Pose(0,0,0));
        waitForStart();
        while(opModeIsActive()) {

            setPower(frontLeft,1);
            setPower(frontRight,1);
            setPower(backLeft,1);
            setPower(backRight,1);
            if(velocityPose.y>maxVelocity){
                maxVelocity = velocityPose.y;
            }
            packet.put("Max Velocity",maxVelocity);
            dashboard.sendTelemetryPacket(packet);
        }
    }}

