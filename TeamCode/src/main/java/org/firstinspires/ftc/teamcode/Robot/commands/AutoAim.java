package org.firstinspires.ftc.teamcode.Robot.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

public class AutoAim extends CommandBase {
    private final Vision vision;
    private final Shooter shooter;
    private final Drive drive;
    private final MecanumDrive mD;

    private static final double MAX_VELOCITY = 2400;
    private static final double MIN_VELOCITY = 1800;

    private final int targetTagId;

    public AutoAim(Vision vision, Shooter shooter, Drive drive, boolean isBlueAlliance) {
        this.vision = vision;
        this.shooter = shooter;
        this.drive = drive;
        this.targetTagId = isBlueAlliance ? 20 : 24;
        mD = drive.getMecanumDrive();

        addRequirements(vision, shooter, drive);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) return;

        double botX = vision.getBotX();
        double botY = vision.getBotY();

        double tagX = (targetTagId == 20) ? 0 : 144;
        double tagY = (targetTagId == 20) ? 144 : 0;

        double dx = tagX - botX;
        double dy = tagY - botY;
        double desiredHeading = Math.toRadians(Math.toDegrees(Math.atan2(dy, dx)));

        Actions.runBlocking(
                mD.actionBuilder(mD.localizer.getPose())
                        .turnTo(desiredHeading)
                        .build()
        );

        double distance = Math.sqrt(dx * dx + dy * dy);
        double velocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * (distance / 67.0);
        velocity = Math.min(MAX_VELOCITY, Math.max(MIN_VELOCITY, velocity));
        shooter.setVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget();
    }
}
