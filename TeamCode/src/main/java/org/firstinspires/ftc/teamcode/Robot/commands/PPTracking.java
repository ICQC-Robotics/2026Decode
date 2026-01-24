package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class PPTracking extends CommandBase {
    private final Turret turret;
    private final Drive d;

    private Robot.Alliance alliance;

    /*
     * far triangle
     * (48,0), (72,24), (96,0)
     *
     * close triangle
     * (15,128), (72,72), (129,128)
     */

    private static final double FAR_AX = 48,  FAR_AY = 0;
    private static final double FAR_BX = 72,  FAR_BY = 24;
    private static final double FAR_CX = 96,  FAR_CY = 0;

    private static final double CLOSE_AX = 15,  CLOSE_AY = 128;
    private static final double CLOSE_BX = 72,  CLOSE_BY = 72;
    private static final double CLOSE_CX = 129, CLOSE_CY = 128;

    private static final double ZONE_HYST = 1.0;
    private boolean trackingEnabled = false;

    //70in: 3700 0.1
    //30in: 3400 0.4
    public double offset = 0;


    double TARGET_X = FieldConstants.BLUE_GOAL_X;
    double TARGET_Y = FieldConstants.BLUE_GOAL_Y;

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
        if (alliance == Robot.Alliance.RED) {
            TARGET_X = FieldConstants.RED_GOAL_X;
            TARGET_Y = FieldConstants.RED_GOAL_Y;
        }
        trackingEnabled = isInsideEitherTriangle(d.getX(), d.getY(), 0.0);
    }
    @Override
    public void execute() {
        double x = d.getX();
        double y = d.getY();

        if (trackingEnabled) {
            trackingEnabled = isInsideEitherTriangle(x, y, ZONE_HYST);
        } else {
            trackingEnabled = isInsideEitherTriangle(x, y, 0.0);
        }

        if (!trackingEnabled) {
            turret.holdCurrentAngle();
            return;
        }

        double targetDeg = turretAngleDeg(x, y, TARGET_X, TARGET_Y, d.getHeading());
        turret.setTargetDeg(targetDeg);
    }


    public double turretAngleDeg(double x, double y,
                                        double targetX, double targetY,
                                        double headingRad) {

        double dx = targetX - x;
        double dy = targetY - y;

        double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg = Math.toDegrees(headingRad);
        double deflectionDeg = wrap180(bearingDeg - headingDeg);
        double turretDeg = wrap360(135.0 - deflectionDeg + offset);

        return turretDeg;
    }

    private boolean isInsideEitherTriangle(double x, double y, double margin) {
        return pointInTriangleWithMargin(
                x, y,
                FAR_AX, FAR_AY,
                FAR_BX, FAR_BY,
                FAR_CX, FAR_CY,
                margin
        ) || pointInTriangleWithMargin(
                x, y,
                CLOSE_AX, CLOSE_AY,
                CLOSE_BX, CLOSE_BY,
                CLOSE_CX, CLOSE_CY,
                margin
        );
    }

    private boolean pointInTriangleWithMargin(
            double px, double py,
            double ax, double ay,
            double bx, double by,
            double cx, double cy,
            double margin
    ) {
        double s1 = signedDistanceToEdge(px, py, ax, ay, bx, by);
        double s2 = signedDistanceToEdge(px, py, bx, by, cx, cy);
        double s3 = signedDistanceToEdge(px, py, cx, cy, ax, ay);

        boolean allNonNeg = (s1 >= -margin) && (s2 >= -margin) && (s3 >= -margin);
        boolean allNonPos = (s1 <=  margin) && (s2 <=  margin) && (s3 <=  margin);

        return allNonNeg || allNonPos;
    }

    private double signedDistanceToEdge(double px, double py,
                                        double ax, double ay,
                                        double bx, double by) {
        double abx = bx - ax;
        double aby = by - ay;
        double apx = px - ax;
        double apy = py - ay;

        double cross = abx * apy - aby * apx;
        double len = Math.hypot(abx, aby);
        if (len < 1e-9) return 0.0;
        return cross / len;
    }

    static double wrap360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    public void incDeg()
    {
        offset++;
    }
    public void decDeg()
    {
        offset--;
    }


    static double wrap180(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
}

