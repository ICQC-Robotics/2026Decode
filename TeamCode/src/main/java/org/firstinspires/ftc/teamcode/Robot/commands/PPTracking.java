package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

public class PPTracking extends CommandBase {
    private final Turret turret;
    private final Drive d;

    private static final double TARGET_X = 19;
    private static final double TARGET_Y = 131;
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

        // Vector from robot to target in field coords
        double dx = TARGET_X - x;
        double dy = TARGET_Y - y;

        // Field-centric bearing to target (radians)
        double bearingRad = Math.atan2(dy, dx);

        // Robot-relative turret angle (radians)
        double turretAngleRad = normalizeRad(bearingRad - headingRad);

        // Convert to degrees for the turret API
        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        turret.setTargetDeg(turretAngleDeg);


    }

    private double normalizeRad(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

}

