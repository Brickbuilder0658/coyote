package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "FindInPerTick")

public class FindInPerTick extends Coyote {
    public final FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {

        initialize(new Pose(0,0,0));
        waitForStart();
        while(opModeIsActive()) {
            packet.put("ticksLeft",(deadLeft.getCurrentPosition()));
            packet.put("ticksRight",(deadRight.getCurrentPosition()));
            dashboard.sendTelemetryPacket(packet);
        }
    }}

