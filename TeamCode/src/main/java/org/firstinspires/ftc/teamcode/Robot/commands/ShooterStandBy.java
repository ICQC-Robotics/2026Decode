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

    public ShooterStandBy(Shooter shooter, Drive drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        double d = AutoAim.calculateDistanceIn(drive);
        shooter.standbyForDistance(d);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
