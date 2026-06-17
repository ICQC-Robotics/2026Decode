package org.firstinspires.ftc.teamcode.Tuner;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

import java.util.List;

@TeleOp(name = "LL Calibrate Zones")
public class LLCalibrate extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "ll");  // config name = "ll"
        ll.setPollRateHz(100);
        Vision vision = new Vision(ll);   // starts LL on GREEN_PIPELINE

        int pipeline = Vision.GREEN_PIPELINE;

        telemetry.addLine("Press START.  X = green,  B = purple");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && pipeline != Vision.GREEN_PIPELINE) {
                pipeline = Vision.GREEN_PIPELINE;  ll.pipelineSwitch(pipeline);
            }
            if (gamepad1.b && pipeline != Vision.PURPLE_PIPELINE) {
                pipeline = Vision.PURPLE_PIPELINE; ll.pipelineSwitch(pipeline);
            }

            telemetry.addData("PIPELINE", pipeline == Vision.GREEN_PIPELINE ? "GREEN (0)" : "PURPLE (1)");
            telemetry.addLine("X=green   B=purple");
            telemetry.addLine("-----------------------------");

            LLResult r = ll.getLatestResult();
            if (r != null && r.isValid()) {
                List<LLResultTypes.ColorResult> results = r.getColorResults();
                telemetry.addData("targets seen", results.size());
                int i = 0;
                for (LLResultTypes.ColorResult c : results) {
                    double tx = c.getTargetXDegrees();
                    telemetry.addData("  blob " + i++,
                            "tx=%.2f  ta=%.3f  -> zone %d",
                            tx, c.getTargetArea(), vision.debugBucket(tx));
                }
            } else {
                telemetry.addLine("NO VALID TARGET");
            }
            telemetry.update();
            sleep(50);
        }
    }
}