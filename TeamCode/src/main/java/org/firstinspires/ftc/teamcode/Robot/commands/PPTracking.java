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
        Pose target = FieldConstants.goalAimPointForAlliance(alliance == Robot.Alliance.BLUE);
        TARGET_X = target.getX();
        TARGET_Y = target.getY();
    }

    @Override
    public void execute() {
        turret.setTargetDeg(
                turretAngleDeg(d.getX(), d.getY(), TARGET_X, TARGET_Y, d.getHeading())
        );

        // Turret diagnostics: if Actual lags Target by a few deg at one heading,
        // it's P-only/cable drag. If Target sits at 0 or 270, it's the travel limit.
        d.telemetry.addData("Turret Target", turret.getTargetDeg());
        d.telemetry.addData("Turret Actual", turret.getAngleDeg());
        d.telemetry.addData("Turret Error",  turret.getTargetDeg() - turret.getAngleDeg());
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

        // Turret mapping:
        // 135 = straight forward, and increasing turret angle turns RIGHT (CW)
        // so: right deflection (negative) -> turret angle increases
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

