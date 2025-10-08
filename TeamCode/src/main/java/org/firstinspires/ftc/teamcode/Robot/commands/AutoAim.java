package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

public class AutoAim extends CommandBase {
    private final Vision vision;
    private final Shooter shooter;
    private final Drive drive;

    private static final double kP_TURN = 0.02;
    private static final double kP_RANGE = 0.04;
    private static final double IDEAL_DISTANCE = 60.0;
    private static final double MAX_VELOCITY = 2400;
    private static final double MIN_VELOCITY = 1800;

    private final int targetTagId;

    public AutoAim(Vision vision, Shooter shooter, Drive drive, boolean isBlueAlliance) {
        this.vision = vision;
        this.shooter = shooter;
        this.drive = drive;
        this.targetTagId = isBlueAlliance ? 20 : 24;

        addRequirements(vision, shooter, drive);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) return;

        double botX = vision.getBotX();
        double botY = vision.getBotY();
        double botHeading = vision.getBotHeading();

        double tagX, tagY;
        if (targetTagId == 20) {
            tagX = 0;
            tagY = 144;
        } else {
            tagX = 144;
            tagY = 0;
        }

        double dx = tagX - botX;
        double dy = tagY - botY;

        double desiredHeading = Math.toDegrees(Math.atan2(dy, dx));
        double headingError = normalizeAngle(desiredHeading - botHeading);

        double distance = Math.sqrt(dx * dx + dy * dy);

        double turnCmd = kP_TURN * headingError;
        double rangeError = distance - IDEAL_DISTANCE;
        double forwardCmd = kP_RANGE * rangeError;

        if (Math.abs(headingError) < 1.0) turnCmd = 0;
        if (Math.abs(rangeError) < 1.0) forwardCmd = 0;

        double fRPower = forwardCmd - turnCmd;
        double fLPower = forwardCmd + turnCmd;
        double bRPower = forwardCmd - turnCmd;
        double bLPower = forwardCmd + turnCmd;

        double max = Math.max(1.0, Math.max(Math.abs(fRPower), Math.max(Math.abs(fLPower), Math.max(Math.abs(bRPower), Math.abs(bLPower)))));

        fRPower /= max;
        fLPower /= max;
        bRPower /= max;
        bLPower /= max;

        drive.getFr().setPower(fRPower);
        drive.getFl().setPower(fLPower);
        drive.getBr().setPower(bRPower);
        drive.getBl().setPower(bLPower);

        double velocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * (distance / 120.0);
        velocity = Math.min(MAX_VELOCITY, Math.max(MIN_VELOCITY, velocity));

        shooter.setVelocity(velocity);
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    @Override
    public boolean isFinished() {
        double headingError = Math.abs(normalizeAngle(vision.getBotHeading()));
        double distance = Math.sqrt(Math.pow(vision.getBotX(), 2) + Math.pow(vision.getBotY(), 2));
        return headingError < 1.0 && Math.abs(distance - IDEAL_DISTANCE) < 1.0;
    }
}
