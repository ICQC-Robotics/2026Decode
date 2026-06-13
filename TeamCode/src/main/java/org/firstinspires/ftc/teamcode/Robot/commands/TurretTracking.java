package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

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
 *
 * Logging: each run writes a CSV to /sdcard/FIRST/turret_log_<timestamp>.csv.
 * Download it from the Control Hub web UI (port 8080 → "Download Logs") or via ADB.
 * Columns: time_ms, ticks, turret_rel_deg, robot_heading_deg, deflection_deg,
 *           robot_x, robot_y, raw_deg, smoothed_deg
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

    // Deflection stored here so execute() can log it without recomputing.
    private double lastDeflectionDeg = 0.0;

    // ── logging ────────────────────────────────────────────────────────────────
    private BufferedWriter log;
    private long           startMs;
    private int            logRow;

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

        openLog();
    }

    @Override
    public void execute() {
        double x          = drive.getX();
        double y          = drive.getY();
        double headingRad = drive.getHeading();

        double raw    = computeGoalAngleDeg(x, y, headingRad);   // also sets lastDeflectionDeg
        double target = Turret.clampDeg(raw + offset);
        smoothed += FILTER_ALPHA * (target - smoothed);
        turret.setTargetDeg(smoothed);

        writeLine(x, y, headingRad, raw);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        closeLog();
    }

    // ── angle geometry ─────────────────────────────────────────────────────────

    private double computeGoalAngleDeg(double x, double y, double headingRad) {
        double turretX = x + Turret.FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + Turret.FORWARD_OFFSET_IN * Math.sin(headingRad);

        double dx = goalX - turretX;
        double dy = goalY - turretY;

        double bearingDeg    = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg    = Math.toDegrees(headingRad);
        lastDeflectionDeg    = wrapHalf(bearingDeg - headingDeg);

        // 135° = straight forward; CW (increasing angle) = turning right.
        return 135.0 - lastDeflectionDeg;
    }

    private static double wrapHalf(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    // ── logging ────────────────────────────────────────────────────────────────

    private void openLog() {
        startMs = System.currentTimeMillis();
        logRow  = 0;
        try {
            String path = "/sdcard/FIRST/turret_log_" + startMs + ".csv";
            log = new BufferedWriter(new FileWriter(path));
            // Metadata so the file is self-describing when read later.
            log.write("# alliance=" + alliance
                    + " goal=(" + goalX + "," + goalY + ")"
                    + " started=" + startMs + "\n");
            log.write("time_ms,ticks,turret_rel_deg,robot_heading_deg,deflection_deg,"
                    + "robot_x,robot_y,raw_deg,smoothed_deg\n");
            log.flush();
        } catch (IOException e) {
            log = null;   // logging unavailable; robot continues normally
        }
    }

    private void writeLine(double x, double y, double headingRad, double raw) {
        if (log == null) return;
        try {
            long   ms           = System.currentTimeMillis() - startMs;
            int    ticks        = turret.getMotorTicks();
            // Relative to forward: 0° = straight ahead, -135° = full left, +135° = full right.
            double turretRelDeg = turret.getAngleDeg() - Turret.FORWARD_DEG;
            double headingDeg   = wrapHalf(Math.toDegrees(headingRad));

            log.write(ms + "," + ticks + ","
                    + fmt(turretRelDeg)       + ","
                    + fmt(headingDeg)         + ","
                    + fmt(lastDeflectionDeg)  + ","
                    + fmt(x)                  + ","
                    + fmt(y)                  + ","
                    + fmt(raw)                + ","
                    + fmt(smoothed)           + "\n");

            // Flush every 50 rows so data survives an unclean shutdown.
            if (++logRow % 50 == 0) log.flush();
        } catch (IOException e) {
            log = null;
        }
    }

    private void closeLog() {
        if (log == null) return;
        try { log.close(); } catch (IOException ignored) {}
        log = null;
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    // ── driver trim ────────────────────────────────────────────────────────────

    public void incDeg()        { offset += 3.0; }
    public void decDeg()        { offset -= 3.0; }
    public void resetDegOffset(){ offset  = 0.0; }
}
