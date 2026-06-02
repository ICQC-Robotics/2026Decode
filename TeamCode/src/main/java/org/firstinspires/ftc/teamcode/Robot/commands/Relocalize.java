package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Relocalize extends CommandBase {

    private final Drive drive;
    private final Vision limelight;
    private final Turret turret;

    private final double maxDistIn;
    private final double maxHeadingDeg;

    public Relocalize(Drive drive, Vision limelight, Turret turret) {
        this(drive, limelight, turret, 3, 10);
    }

    public Relocalize(Drive drive, Vision limelight, Turret turret,
                                   double maxDistIn, double maxHeadingDeg) {
        this.drive = drive;
        this.limelight = limelight;
        this.turret = turret;
        this.maxDistIn = maxDistIn;
        this.maxHeadingDeg = maxHeadingDeg;
    }

    @Override
    public void initialize() {

        Pose llPose = limelight.getBotposeMT1(turret.getAngleDeg());

        if (llPose == null) {
            drive.telemetry.addLine("llPose NULL");
            return;
        }

        Pose est = drive.follower.getPose();

        double dx = llPose.getX() - est.getX();
        double dy = llPose.getY() - est.getY();
        double dist = Math.hypot(dx, dy);

        double estDeg = Math.toDegrees(est.getHeading());
        double llDeg  = Math.toDegrees(llPose.getHeading());
        double dHeadDeg = wrap(llDeg - estDeg);

//        if (dist > maxDistIn) return;
//        if (Math.abs(dHeadDeg) > maxHeadingDeg) return;

        drive.follower.setPose(llPose);
        Robot.LAST_POSE = llPose.copy();

        drive.telemetry.addLine("Relocalized");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    private static double wrap(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
}
