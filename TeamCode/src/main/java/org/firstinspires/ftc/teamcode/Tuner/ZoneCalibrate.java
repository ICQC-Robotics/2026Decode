package org.firstinspires.ftc.teamcode.Tuner;

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
                .addProcessor(zoneProcessor)
                .build();

        telemetry.addLine("Press START, then open Camera Stream on the Driver Station.");
        telemetry.addLine("The 3 yellow boxes are the computed zone boundaries.");
        telemetry.addLine("Slide an artifact through each box; tune ArtifactZoneProcessor's");
        telemetry.addLine("HORIZONTAL_FOV_DEG / HSV thresholds until boxes line up with the");
        telemetry.addLine("real zones and density% only rises when an artifact is inside.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double[] pct = zoneProcessor.getRegionDensityPct();  // [left, mid, right]
            telemetry.addLine("=== on-screen density % (alliance-independent) ===");
            telemetry.addData("left  (zone3 if BLUE, zone1 if RED)", "%.1f%%", pct[0]);
            telemetry.addData("mid   (zone2, either alliance)",       "%.1f%%", pct[1]);
            telemetry.addData("right (zone1 if BLUE, zone3 if RED)", "%.1f%%", pct[2]);
            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}
