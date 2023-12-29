package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "TestAngular")

public class TestAngular extends Coyote {
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    public void testAngular() {
        frontRight.setPower(-1);  // front
        frontLeft.setPower(1);    // left
        backRight.setPower(-1);    // right
        backLeft.setPower(1);
        while (opModeIsActive()) {

            Pose current = new Pose(deadPerp.getCurrentPosition(), 0, positiveWrap((getCurrentPose().angle)));
            current.angle = Math.toDegrees(current.angle);
            if (current.angle > previousAngle) {
                spinCounter += 1;
            }
            previousAngle = (current.angle);
            packet.put("ratio", (current.x / (-spinCounter * 360 + current.angle)));
            dashboard.sendTelemetryPacket(packet);
        }
    }

    @Override
    public void runOpMode() {

        initialize(new Pose(0,0,0));
        waitForStart();
        testAngular();
    }}

