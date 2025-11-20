package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim.Positions;

public class ShooterStandBy extends CommandBase {

    private final Shooter shooter;
    private final Vision vision;

    private final double limelightHeight = 17.0;
    private final double aprilTagHeight = 29.5;
    private final double limelightPitch = 23.0;
    private final double minV = 1950, maxV = 2450;
    private final double minD = 30, maxD = 70;

    public ShooterStandBy(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPIDF(0.075, 0, 0.005, 0.53);
        shooter.setMagazineCover(Positions.CLOSED_COVER.getPos());
    }

    @Override
    public void execute() {

        double v = calculateVelocity();
        if (v < 2000) v = 2000;

        shooter.setVelocity(v);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double calculateVelocity() {
        if (Double.isNaN(vision.getTy())) return 0;

        double actualHeight = aprilTagHeight - limelightHeight;
        double angle = limelightPitch + vision.getTy();
        double d = actualHeight / Math.tan(Math.toRadians(angle));
        if (d < 30) d = 30;

        return minV + (maxV - minV) * (d - minD) / (maxD - minD);
    }
}
