// NEW FILE: ClampedPPTracking.java
// package should match your commands package
package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * PPTracking but with a clamp window (only used where you schedule this command).
 * Keeps the angle continuous near current turret angle to avoid wrap jumps, then clamps.
 */
public class ClampedPPTracking extends CommandBase {
    private final Turret turret;
    private final Drive d;
    private final Robot.Alliance alliance;

    private final double minDeg;
    private final double maxDeg;

    // tune / match your PPTracking
    private static final double TURRET_FORWARD_OFFSET_IN = 3;

    private double targetX = FieldConstants.BLUE_GOAL_AIM_X;
    private double targetY = FieldConstants.BLUE_GOAL_AIM_Y;

    // optional manual offset like PPTracking
    public double offset = 0;

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

        // IMPORTANT: don't call holdCurrentAngle() here if it schedules/locks turret
        // turret.holdCurrentAngle();
    }

    @Override
    public void execute() {
        double desired = turretAngleDeg(
                d.getX(), d.getY(),
                targetX, targetY,
                d.getHeading()
        );

        double current = turret.getAngleDeg();

        // make desired continuous near current (shortest-path)
        double desiredCont = current + wrap180(desired - current);

        // clamp
        double clamped = clamp(desiredCont, minDeg, maxDeg);

        turret.setTargetDeg(wrap360(clamped));
    }

    /** Same math as PPTracking: returns 0..360 */
    public double turretAngleDeg(double x, double y,
                                 double targetX, double targetY,
                                 double headingRad) {

        // shift origin from robot center -> turret position (field coords)
        double turretX = x + TURRET_FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + TURRET_FORWARD_OFFSET_IN * Math.sin(headingRad);

        double dx = targetX - turretX;
        double dy = targetY - turretY;

        // Field/global bearing to target: 0=right, 90=up, +CCW
        double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading in same convention
        double headingDeg = Math.toDegrees(headingRad);

        // Required turret deflection relative to robot forward
        double deflectionDeg = wrap180(bearingDeg - headingDeg);

        // Turret mapping:
        // 135 = straight forward, and increasing turret angle turns RIGHT (CW)
        double turretDeg = wrap360(135.0 - deflectionDeg + offset);

        return turretDeg;
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

    // optional offset helpers (same idea as PPTracking)
    public void incDeg() { offset += 3; }
    public void decDeg() { offset -= 3; }
    public void resetDegOffset() { offset = 0; }
}
