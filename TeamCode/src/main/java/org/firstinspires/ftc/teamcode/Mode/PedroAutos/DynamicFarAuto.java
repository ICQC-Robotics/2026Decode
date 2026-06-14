package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * Dynamic Far Autonomous — DECODE 2025-2026.
 *
 * Uses the Limelight (mounted on the REAR-facing intake) to find the highest-density
 * cluster of artifacts, drives there leading with the intake to collect (intake always
 * on), returns to a shoot spot and fires (turret auto-aims), then re-scans and repeats.
 *
 * Init: place the robot near (48, 24), then PUSH it to the real start during INIT —
 * odometry tracks the push and the first path is built from the live pose at run.
 *
 * Alliance select before START:  X = BLUE (default),  B = RED.
 */
@Autonomous(name = "DynamicFarAuto", group = "Far")
public class DynamicFarAuto extends CommandOpMode {

    Robot negabot;
    boolean isBlue = true;

    // ── Tuning (blue frame; mirrored for red) ───────────────────────────────
    // Init/push-to-start spot. INIT_HEADING_DEG MUST match how you place the robot.
    static final double INIT_X = 48.0, INIT_Y = 24.0, INIT_HEADING_DEG = 0.0;
    static final double SHOOT_X = 72.0, SHOOT_Y = 84.0;          // turret auto-aims, heading is free
    static final double FALLBACK_X = 60.0, FALLBACK_Y = 60.0;    // where to go if the camera sees nothing

    static final double SHOOT_VELOCITY = 3700.0;   // far flywheel RPM
    static final double SHOOT_HOOD     = 0.7;       // far hood
    static final double INTAKE_ON      = -1.0;      // intake direction (collect)

    static final double COVER_OPEN = 1.0, COVER_CLOSE = 0.1;
    static final long   BURST_MS = 1000;            // feed window per volley
    static final long   COLLECT_DWELL_MS = 400;     // sit on the cluster to finish collecting
    static final long   SHOOT_SETTLE_MS = 350;      // let turret + flywheel settle before firing

    static final int    CYCLES = 3;
    static final double FIELD_MIN = 8.0, FIELD_MAX = 136.0;   // clamp camera target inside the field

    @Override
    public void initialize() {
        // Created up front so the Pinpoint tracks the push during INIT.
        negabot = new Robot(hardwareMap, telemetry, initPose());
        Robot.Alliance shown = Robot.Alliance.BLUE;

        // ── Alliance select + push-to-start ───────────────────────────────
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            Robot.Alliance sel = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
            if (sel != shown) {                              // alliance changed -> re-zero init pose
                negabot.drive.follower.setPose(initPose());
                shown = sel;
            }
            negabot.drive.follower.update();                 // track the push
            Pose live = negabot.drive.follower.getPose();
            telemetry.addLine("=== SELECT ALLIANCE (X/B), THEN PUSH TO START ===");
            telemetry.addData("Selected", isBlue ? ">>> BLUE <<<" : ">>>  RED <<<");
            telemetry.addData("Live pose", String.format("x=%.1f y=%.1f h=%.0f",
                    live.getX(), live.getY(), Math.toDegrees(live.getHeading())));
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        Robot.ALLIANCE = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
        negabot.turret.resetEncoder();

        Follower f = negabot.drive.follower;
        Pose shoot = sp(SHOOT_X, SHOOT_Y, 0);

        SequentialCommandGroup routine = new SequentialCommandGroup();
        for (int i = 0; i < CYCLES; i++) {
            routine.addCommands(
                driveToDensest(f),                          // dynamic: drive to the densest cluster
                new WaitCommand(COLLECT_DWELL_MS),
                driveTo(f, shoot.getX(), shoot.getY()),     // return to the shoot spot
                shoot()                                     // aim + fire
            );
        }

        negabot.schedule(
            new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
            new InstantCommand(() -> negabot.shooter.setHoodPosition(SHOOT_HOOD)),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE)),
            new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),   // intake on the whole auto
            routine
        );
    }

    // ── Dynamic command: drive to the densest artifact cluster ──────────────
    Command driveToDensest(Follower f) {
        return new CommandBase() {
            @Override public void initialize() {
                Pose r = f.getPose();
                // Camera rides the REAR intake, so it looks backward: heading + 180°.
                Pose camPose = new Pose(r.getX(), r.getY(), r.getHeading() + Math.PI);
                Pose seen = negabot.vision.getArtifactFieldPose(camPose);   // densest cluster, or null

                double tx, ty;
                if (seen != null) {
                    tx = clamp(seen.getX(), FIELD_MIN, FIELD_MAX);
                    ty = clamp(seen.getY(), FIELD_MIN, FIELD_MAX);
                    telemetry.addData("Densest cluster", "x=%.1f y=%.1f", tx, ty);
                } else {
                    Pose fb = sp(FALLBACK_X, FALLBACK_Y, 0);
                    tx = fb.getX(); ty = fb.getY();
                    telemetry.addLine("No cluster seen — driving to fallback");
                }
                telemetry.update();

                // Lead with the rear intake: face AWAY from the cluster and back into it.
                double intakeIntoTarget = Math.atan2(ty - r.getY(), tx - r.getX()) + Math.PI;
                PathChain path = f.pathBuilder()
                        .addPath(new BezierLine(new Pose(r.getX(), r.getY()), new Pose(tx, ty)))
                        .setLinearHeadingInterpolation(r.getHeading(), intakeIntoTarget)
                        .build();
                f.followPath(path, 1.0, true);
            }
            @Override public boolean isFinished() { return !f.isBusy(); }
        };
    }

    // ── Drive to a fixed field point (straight line from the live pose) ─────
    Command driveTo(Follower f, double tx, double ty) {
        return new CommandBase() {
            @Override public void initialize() {
                Pose r = f.getPose();
                double headingToTarget = Math.atan2(ty - r.getY(), tx - r.getX());
                PathChain path = f.pathBuilder()
                        .addPath(new BezierLine(new Pose(r.getX(), r.getY()), new Pose(tx, ty)))
                        .setLinearHeadingInterpolation(r.getHeading(), headingToTarget)
                        .build();
                f.followPath(path, 1.0, true);
            }
            @Override public boolean isFinished() { return !f.isBusy(); }
        };
    }

    // ── Aim the turret at the goal and fire one volley ──────────────────────
    Command shoot() {
        return new SequentialCommandGroup(
            new InstantCommand(this::aimTurretAtGoal),
            new WaitCommand(SHOOT_SETTLE_MS),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
            new WaitCommand(BURST_MS),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    /** Aim the turret at the alliance goal from the robot's live pose (any heading). */
    void aimTurretAtGoal() {
        Pose p = negabot.drive.follower.getPose();
        if (p == null) return;
        double h = p.getHeading();
        double tx = p.getX() + Turret.FORWARD_OFFSET_IN * Math.cos(h) - Turret.LATERAL_OFFSET_IN * Math.sin(h);
        double ty = p.getY() + Turret.FORWARD_OFFSET_IN * Math.sin(h) + Turret.LATERAL_OFFSET_IN * Math.cos(h);
        Pose goal = FieldConstants.goalAimPointForAlliance(isBlue);
        double bearingDeg    = Math.toDegrees(Math.atan2(goal.getY() - ty, goal.getX() - tx));
        double deflectionDeg = wrap180(bearingDeg - Math.toDegrees(h));
        negabot.turret.setTargetDeg(Turret.clampDeg(135.0 - deflectionDeg));
    }

    @Override
    public void run() {
        if (negabot == null) return;
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }

    // ── helpers ─────────────────────────────────────────────────────────────

    private Pose initPose() {
        return sp(INIT_X, INIT_Y, INIT_HEADING_DEG);
    }

    /** Alliance-aware pose: blue direct, red mirrored across x = 72 (heading θ → 180−θ). */
    private Pose sp(double x, double y, double headingDeg) {
        return isBlue
                ? new Pose(x, y, Math.toRadians(headingDeg))
                : new Pose(144.0 - x, y, Math.toRadians(((180.0 - headingDeg) % 360.0 + 360.0) % 360.0));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double wrap180(double a) {
        a = ((a + 180.0) % 360.0 + 360.0) % 360.0;
        return a - 180.0;
    }
}
