package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "FindMotorDirections")

public class FindMotorDirections extends Coyote {
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {

        initialize(new Pose(0,0,0));
        waitForStart();
        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        backRight.setPower(.5);
        backLeft.setPower(.5);
        sleep(2500);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }}

