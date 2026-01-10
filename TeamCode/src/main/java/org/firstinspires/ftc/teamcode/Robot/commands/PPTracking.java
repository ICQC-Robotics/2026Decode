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

    //70in: 3700 0.1
    //30in: 3400 0.4
    private static final double DEADBAND_DEG = 1;//tune this
    private static final double FORWARD_DEG = 135;

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
    }
    @Override
    public void execute() {
        turret.setTargetDeg(turretAngleDeg(d.getX(), d.getY(), TARGET_X, TARGET_Y, d.getHeading()));
    }

    public static double turretAngleDeg(double x, double y,
                                        double targetX, double targetY,
                                        double headingRad) {

        double dx = targetX - x;
        double dy = targetY - y;

        // Field/global bearing to target: 0=right, 90=up, +CCW
        double bearingDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading in same convention
        double headingDeg = Math.toDegrees(headingRad);

        // Required turret deflection relative to robot forward:
        // + = target is to robot's left, - = to robot's right
        double deflectionDeg = wrap180(bearingDeg - headingDeg);

        // Turret mapping:
        // 135 = straight forward, and increasing turret angle turns RIGHT (CW)
        // so: right deflection (negative) -> turret angle increases
        double turretDeg = wrap360(135.0 - deflectionDeg );

        return turretDeg;
    }

    static double wrap360(double a) {
        a %= 360.0;
        if (a < 0) a += 360.0;
        return a;
    }

    static double wrap180(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }

    private double normalizeRad(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

}

