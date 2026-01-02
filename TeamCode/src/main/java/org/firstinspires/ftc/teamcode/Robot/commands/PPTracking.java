package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class PPTracking extends CommandBase {
    private final Turret turret;
    private final Drive d;

    private static final double DEADBAND_DEG = 1;//tune this
    private static final double FORWARD_DEG = 135;

    double TARGET_X = (Robot.ALLIANCE == Robot.Alliance.BLUE)? FieldConstants.BLUE_GOAL_X: FieldConstants.RED_GOAL_X;
    double TARGET_Y = (Robot.ALLIANCE == Robot.Alliance.BLUE)? FieldConstants.BLUE_GOAL_Y: FieldConstants.RED_GOAL_Y;

    public PPTracking(Turret turret, Drive d) {
        this.turret = turret;
        this.d = d;
        addRequirements(turret);
    }
    @Override
    public void initialize() {
        turret.holdCurrentAngle();
    }
    @Override
    public void execute() {
        double x = d.getX();
        double y = d.getY();
        double headingRad = d.getHeading();

        double dx = TARGET_X - x;
        double dy = TARGET_Y - y;

        double bearingRad = Math.atan2(dy, dx);
        double turretAngleDeg = FORWARD_DEG + Math.toDegrees(normalizeRad(bearingRad - headingRad));

        if (Math.abs(turretAngleDeg - turret.getAngleDeg()) > DEADBAND_DEG)
            turret.setTargetDeg(turretAngleDeg);    }

    private double normalizeRad(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

}

