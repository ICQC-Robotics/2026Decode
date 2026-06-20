package org.firstinspires.ftc.teamcode.Tuner;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Vision.ArtifactZoneProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Zone Calibrate (Webcam)")
public class ZoneCalibrate extends LinearOpMode {

    @Override
    public void runOpMode() {
        ArtifactZoneProcessor zoneProcessor = new ArtifactZoneProcessor();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "zoneCam"))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(zoneProcessor)
                .build();

        boolean isBlue = true;

        telemetry.addLine("Press START, then open Camera Stream on the Driver Station.");
        telemetry.addLine("The 3 yellow boxes (labeled z1/z2/z3) are the zone boundaries.");
        telemetry.addLine("X = BLUE, B = RED -- watch the boxes mirror when you switch.");
        telemetry.addLine("Slide an artifact through each box; tune the HSV thresholds until");
        telemetry.addLine("density% only rises when an artifact is actually inside its box.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            zoneProcessor.setAlliance(isBlue);

            double[] pct = zoneProcessor.getZoneDensityPct();  // index 1..3
            telemetry.addData("Alliance (X=blue, B=red)", isBlue ? "BLUE" : "RED");
            telemetry.addData("density z1/z2/z3", "%.1f%% / %.1f%% / %.1f%%", pct[1], pct[2], pct[3]);
            telemetry.addData("Vision FPS", "%.1f", portal.getFps());
            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}
