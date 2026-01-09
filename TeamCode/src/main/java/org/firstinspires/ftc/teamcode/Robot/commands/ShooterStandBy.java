package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;

/**
 * Standby spin-up that continuously recomputes shooter velocity
 * from current robot pose, using the same distance->RPM mapping as AutoAim.
 */
public class ShooterStandBy extends CommandBase {

    private final Shooter shooter;
    private final Drive drive;

    // Match AutoAim's mapping window
    private static final double MIN_DIST = 30;
    private static final double MAX_DIST = 130;

    // Use AutoAim's published velocity bounds so you only tune in one place.
    private static final double MIN_V = AutoAim.MIN_V;
    private static final double MAX_V = AutoAim.MAX_V;

    public ShooterStandBy(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double d = calculateDistanceIn(drive);
        double rpm = clamp(calculateRpm(d), MIN_V, MAX_V);

        // If you also want standby to track hood like AutoAim, uncomment:
        // shooter.setHoodPos(setHood(d));

        shooter.setVelocity(rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double calculateDistanceIn(Drive drive) {
        Pose robot = drive.follower.getPose();
        if (robot == null) return MIN_DIST;

        double goalX = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_X : FieldConstants.RED_GOAL_X;
        double goalY = (Robot.ALLIANCE == Robot.Alliance.BLUE) ? FieldConstants.BLUE_GOAL_Y : FieldConstants.RED_GOAL_Y;

        double dx = goalX - robot.getX();
        double dy = goalY - robot.getY();
        return Math.hypot(dx, dy);
    }

    // linear interp
    private double calculateRpm(double distanceIn) {
        if (distanceIn < MIN_DIST) distanceIn = MIN_DIST;
        if (distanceIn > MAX_DIST) distanceIn = MAX_DIST;
        return MIN_V + (MAX_V - MIN_V) * (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
    }

    @SuppressWarnings("unused")
    private double setHood(double distanceIn) {
        double hoodNear = 0.4;
        double hoodFar  = 0.1;

        double t = (distanceIn - MIN_DIST) / (MAX_DIST - MIN_DIST);
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        return hoodNear + t * (hoodFar - hoodNear);
    }

    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
