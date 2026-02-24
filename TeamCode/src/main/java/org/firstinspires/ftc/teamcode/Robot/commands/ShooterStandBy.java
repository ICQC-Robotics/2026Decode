package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;

/**
 * Standby spin-up that continuously recomputes shooter velocity
 * from current robot pose, using AutoAim's distance->RPM lookup table.
 */
public class ShooterStandBy extends CommandBase {

    private final Shooter shooter;
    private final Drive drive;

    // Use AutoAim's published velocity bounds so you only tune in one place.
    private static final double MIN_V = AutoAim.MIN_V;
    private static final double MAX_V = AutoAim.MAX_V;

    public ShooterStandBy(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double d = AutoAim.calculateDistanceIn(drive);
        double rpm = clamp(AutoAim.getRpmForDistance(d), MIN_V, MAX_V);
        shooter.setVelocity(rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    private static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
