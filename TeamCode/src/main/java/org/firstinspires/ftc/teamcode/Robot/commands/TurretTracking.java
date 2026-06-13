package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * Default command: continuously aims the turret at the alliance goal using odometry.
 *
 * The exponential low-pass filter runs on the already-clamped target so the filter
 * state is always inside [MIN_DEG, MAX_DEG]. This means:
 *  - No circular-wrap arithmetic is needed (the range never crosses 0°/360°).
 *  - The filter cannot drift past the physical limits and cause lag when the target
 *    comes back into range (the old unbounded-smoothed bug near the edges).
 *
 * When the goal is outside the turret's physical arc the command clamps to the
 * nearest reachable endpoint, so the turret is always as close as possible.
 */
public class TurretTracking extends CommandBase {

    private static final double FILTER_ALPHA = 0.6;

    private final Turret turret;
    private final Drive  drive;
    private final Robot.Alliance alliance;

    /** Driver-adjustable trim (degrees). Applied before clamping. */
    public double offset = 0.0;

    private double smoothed;
    private double goalX, goalY;

    public TurretTracking(Turret turret, Drive drive, Robot.Alliance alliance) {
        this.turret   = turret;
        this.drive    = drive;
        this.alliance = alliance;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        Pose goal = FieldConstants.goalAimPointForAlliance(alliance == Robot.Alliance.BLUE);
        goalX = goal.getX();
        goalY = goal.getY();
        // Seed from current physical position to prevent a snap on the first loop.
        smoothed = turret.getAngleDeg();
    }

    @Override
    public void execute() {
        double raw    = computeGoalAngleDeg(drive.getX(), drive.getY(), drive.getHeading());
        double target = Turret.clampDeg(raw + offset);
        // Linear interpolation — no circular wrap needed because [MIN_DEG, MAX_DEG]
        // is entirely within [0°, 270°] and never straddles the 0°/360° boundary.
        smoothed += FILTER_ALPHA * (target - smoothed);
        turret.setTargetDeg(smoothed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ── angle geometry ─────────────────────────────────────────────────────────

    private double computeGoalAngleDeg(double x, double y, double headingRad) {
        // Shift from robot centre to turret pivot.
        double turretX = x + Turret.FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + Turret.FORWARD_OFFSET_IN * Math.sin(headingRad);

        double dx = goalX - turretX;
        double dy = goalY - turretY;

        // Bearing to goal in field frame (0° = +X, CCW positive — matches Pedro heading).
        double bearingDeg   = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg   = Math.toDegrees(headingRad);
        // Positive deflection = goal is to the left of robot heading.
        double deflectionDeg = wrapHalf(bearingDeg - headingDeg);

        // 135° = straight forward; CW (increasing angle) = turning right.
        // Subtract deflection: left target → smaller angle, right target → larger angle.
        return 135.0 - deflectionDeg;
    }

    private static double wrapHalf(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    // ── driver trim ────────────────────────────────────────────────────────────

    public void incDeg()       { offset += 3.0; }
    public void decDeg()       { offset -= 3.0; }
    public void resetDegOffset() { offset  = 0.0; }
}
