package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "GetRPMVals", group = ".")
public class GetRPMVals extends OpMode {

    private Robot negabot;

    private double targetRpm = 2000.0;

    private final double limelightHeight = 17.0;
    private final double aprilTagHeight = 29.5;
    private final double limelightPitch = 23.0;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void init() {
        GamepadEx g = new GamepadEx(gamepad1);

        negabot = new Robot(hardwareMap, g, null);

        telemetry.addLine("dpad up/down = +/- 50 RPM");
        telemetry.addLine("dpad right/left =  +/- 200 RPM");
    }

    @Override
    public void loop() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !lastUp) {
            targetRpm += 50;
        }
        if (down && !lastDown) {
            targetRpm -= 50;
        }
        if (right && !lastRight) {
            targetRpm += 200;
        }
        if (left && !lastLeft) {
            targetRpm -= 200;
        }

        if (targetRpm < 1950) targetRpm = 1950;
        if (targetRpm > 3200) targetRpm = 3200;

        negabot.shooter.setVelocity(targetRpm);
        double distance = calculateDistanceInches();
        double actualRpm = negabot.shooter.getVelocity();

        telemetry.addData("target ", "%.0f", targetRpm);
        telemetry.addData("actual ", "%.0f", actualRpm);
        telemetry.addData("d ", "%.1f", distance);
        telemetry.update();

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }

    private double calculateDistanceInches() {
        double ty = negabot.vision.getTy();
        if (Double.isNaN(ty)) return Double.NaN;

        double actualHeight = aprilTagHeight - limelightHeight;
        double angle = limelightPitch + ty;
        return actualHeight / Math.tan(Math.toRadians(angle));
    }
}
