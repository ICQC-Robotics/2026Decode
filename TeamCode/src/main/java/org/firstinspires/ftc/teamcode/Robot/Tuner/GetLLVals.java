package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GetLLVals", group = "Tuning")
public class GetLLVals extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        limelight.start();

        telemetry.addLine("=== Limelight tA Reader ===");
        telemetry.addLine("Center the AprilTag in view to see target area values.");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result == null) {
            telemetry.addLine("No Limelight result yet");
        } else if (!result.isValid()) {
            telemetry.addLine("No valid target detected");
        } else {
            double tA = result.getTa();
            double tx = result.getTx();
            double ty = result.getTy();
            double actualHeight = 29.5 - 17;
            double angle = 23 + ty;
            double distance = actualHeight / Math.tan(Math.toRadians(angle));

            telemetry.addData("tA (raw)", tA);
            telemetry.addData("tx (raw)", tx);
            telemetry.addData("ty (raw)", ty);
            telemetry.addData("distance", distance);

        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
