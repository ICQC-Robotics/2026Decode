package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class PPTracking extends CommandBase {
    private final Turret turret;
    private final Drive d;

    private Robot.Alliance alliance;

    static final double TURRET_FORWARD_OFFSET_IN = 3;

    public double offset = 0;

    // Exponential low-pass filter on the computed goal angle.
    // Localization noise jitters the raw angle ±1° or more per loop; with no
    // derivative term (kD=0) each micro-correction overshoots, causing the
    // random bidirectional error. The filter lets genuine tracking changes
    // through while attenuating high-frequency noise.
    // Tune: increase toward 1.0 for faster response, decrease for more smoothing.
    private static final double FILTER_ALPHA = 0.6;
    private double smoothedAngle = 135.0;

    double TARGET_X = FieldConstants.BLUE_GOAL_AIM_X;
    double TARGET_Y = FieldConstants.BLUE_GOAL_AIM_Y;

    public PPTracking(Turret turret, Drive d, Robot.Alliance alliance) {
        this.turret = turret;
        this.d = d;
        this.alliance = alliance;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        Robot.LAST_TURRET_DEG = 135;
        turret.holdCurrentAngle();
        smoothedAngle = turret.getAngleDeg();
        Pose target = FieldConstants.goalAimPointForAlliance(alliance == Robot.Alliance.BLUE);
        TARGET_X = target.getX();
        TARGET_Y = target.getY();
    }

    @Override
    public void execute() {
        double raw = turretAngleDeg(d.getX(), d.getY(), TARGET_X, TARGET_Y, d.getHeading());
        // wrap180 diff keeps the filter interpolating via the short path across 0/360.
        smoothedAngle = wrap360(smoothedAngle + FILTER_ALPHA * wrap180(raw - smoothedAngle));
        turret.setTargetDeg(smoothedAngle);
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
        double turretDeg = wrap360(135.0 - deflectionDeg + offset);

        return turretDeg;
    }

    public static double wrap360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    public void incDeg() { offset = offset + 3; }
    public void decDeg() { offset = offset - 3; }
    public void resetDegOffset() { offset = 0; }

    public static double wrap180(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
}
