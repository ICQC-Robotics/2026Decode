package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

public class AutoAim extends CommandBase {
    Vision vision;
    Shooter shooter;

    public AutoAim(Vision vision, Shooter shooter) {
        this.vision = vision;
        this.shooter = shooter;

        addRequirements(vision, shooter);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double dy = vision.getTy();
            double x = getDistance(dy);
            double v = getVelocity(x);
            shooter.setVelocity(v);
        } else {
            shooter.setVelocity(0);
        }
    }

    private double getDistance(double dy) {
        double limelightHeight = 0.3;
        double targetHeight = 2.0;
        double limelightAngle = Math.toRadians(30);
        double angleToTarget = limelightAngle + Math.toRadians(dy);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }

    private double getVelocity(double x) {
        double m = 1000;
        double b = 2000;
        return m * x + b;
    }
}
