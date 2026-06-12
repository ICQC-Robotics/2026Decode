package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * PPTracking but with a clamp window (only used where you schedule this command).
 * The exponential filter both smooths out localization noise and handles angle
 * continuity across the 0/360 wrap boundary, replacing the old desiredCont logic.
 */
public class ClampedPPTracking extends CommandBase {
    private final Turret turret;
    private final Drive d;
    private final Robot.Alliance alliance;

    private final double minDeg;
    private final double maxDeg;

    private static final double TURRET_FORWARD_OFFSET_IN = 3;

    private double targetX = FieldConstants.BLUE_GOAL_AIM_X;
    private double targetY = FieldConstants.BLUE_GOAL_AIM_Y;

    public double offset = 0;

    private static final double FILTER_ALPHA = 0.6;
    private double smoothedAngle = 135.0;

    public ClampedPPTracking(Turret turret, Drive d, Robot.Alliance alliance,
                             double minDeg, double maxDeg) {
        this.turret = turret;
        this.d = d;
        this.alliance = alliance;
        this.minDeg = minDeg;
        this.maxDeg = maxDeg;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        Pose target = FieldConstants.goalAimPointForAlliance(alliance == Robot.Alliance.BLUE);
        targetX = target.getX();
        targetY = target.getY();
        smoothedAngle = turret.getAngleDeg();
    }

    @Override
    public void execute() {
        double raw = turretAngleDeg(d.getX(), d.getY(), targetX, targetY, d.getHeading());
        // Shortest-path exponential filter: handles 0/360 wrap and rejects noise.
        smoothedAngle = wrap360(smoothedAngle + FILTER_ALPHA * wrap180(raw - smoothedAngle));
        turret.setTargetDeg(clamp(smoothedAngle, minDeg, maxDeg));
    }

    public double turretAngleDeg(double x, double y,
                                 double targetX, double targetY,
                                 double headingRad) {

        double turretX = x + TURRET_FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + TURRET_FORWARD_OFFSET_IN * Math.sin(headingRad);

        double dx = targetX - turretX;
        double dy = targetY - turretY;

        double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg = Math.toDegrees(headingRad);
        double deflectionDeg = wrap180(bearingDeg - headingDeg);

        // 135 = straight forward, increasing angle turns RIGHT (CW)
        return wrap360(135.0 - deflectionDeg + offset);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static double wrap360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    public static double wrap180(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    public void incDeg() { offset += 3; }
    public void decDeg() { offset -= 3; }
    public void resetDegOffset() { offset = 0; }
}
