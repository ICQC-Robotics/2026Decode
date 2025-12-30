package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

public class ShooterStandBy extends CommandBase {

    private final Shooter shooter;
    private final double rpm;

    private final double limelightHeight = 17.0;
    private final double aprilTagHeight = 29.5;
    private final double limelightPitch = 23.0;
    private final double minV = 1950, maxV = 2450;
    private final double minD = 30, maxD = 70;

   //QuadReg vals
    private double A = 0.0;
    private double B = 0.0;
    private double C = 0.0;

    public ShooterStandBy(Shooter shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(rpm);
    }

//    private double calculateVelocity() {
//        double ty = vision.getTy();
//        if (Double.isNaN(ty)) return minV;
//
//        double actualHeight = aprilTagHeight - limelightHeight;
//        double angle = limelightPitch + ty;
//        double d = actualHeight / Math.tan(Math.toRadians(angle));
//
//        if (d < minD) d = minD;
//        if (d > maxD) d = maxD;
//
//        double v = A * d * d + B * d + C;
//
//        if (v < minV) v = minV;
//        if (v > maxV) v = maxV;
//
//        return v;
//    }
}
