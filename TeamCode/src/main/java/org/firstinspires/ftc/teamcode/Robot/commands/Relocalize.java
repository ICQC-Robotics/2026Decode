package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * Continuously re-syncs the follower's pose (including heading) to the Limelight's
 * AprilTag-based botpose whenever a fresh, sane reading is available.
 *
 * The turret aims purely off drive.follower.getPose() (PPTracking), and odometry
 * heading drifts the more the robot spins/drives - so without periodic correction
 * here, that drift shows up directly as turret misalignment over a match. Run this
 * as a default command so it's always quietly correcting in the background.
 */
public class Relocalize extends CommandBase {

    private static final double RELOCALIZE_PERIOD_S = 0.5;

    private final Drive drive;
    private final Vision limelight;
    private final Turret turret;

    private final double maxDistIn;
    private final double maxHeadingDeg;

    private final ElapsedTime timer = new ElapsedTime();

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
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.seconds() < RELOCALIZE_PERIOD_S) return;
        timer.reset();

        Pose llPose = limelight.getBotposeMT1(turret.getAngleDeg());
        if (llPose == null) return;

        Pose est = drive.follower.getPose();

        double dx = llPose.getX() - est.getX();
        double dy = llPose.getY() - est.getY();
        double dist = Math.hypot(dx, dy);

        double estDeg = Math.toDegrees(est.getHeading());
        double llDeg  = Math.toDegrees(llPose.getHeading());
        double dHeadDeg = wrap(llDeg - estDeg);

        // Reject readings that disagree wildly with odometry - those are far more
        // likely to be a bad/noisy detection than the truth, and blindly trusting
        // them would snap the pose (and the turret aim) to the wrong place.
        if (dist > maxDistIn) return;
        if (Math.abs(dHeadDeg) > maxHeadingDeg) return;

        drive.follower.setPose(llPose);
        Robot.LAST_POSE = llPose.copy();
        drive.telemetry.addLine("Relocalized");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static double wrap(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
}
