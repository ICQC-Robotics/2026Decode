package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * 24-Artifact Close Autonomous — DECODE 2025-2026.
 *
 * Select alliance with gamepad1 before pressing START:
 *   X  →  BLUE  (default)
 *   B  →  RED
 *
 * Cycle order (8 total):
 *   1. Preload     — fire pre-loaded artifacts at (40,90) @ 2950 RPM
 *   2. Middle row  — sweep row at y ≈ 60; return to (52,90) @ 3050 RPM
 *   3. Gate        — sweep gate area; return to (52,90) @ 3050 RPM
 *   4. Top row     — sweep row at y ≈ 84; return to (40,90) @ 2950 RPM
 *   5. Gate        — sweep gate area (second pass); return to (52,90) @ 3050 RPM
 *   6. Bottom row  — sweep row at y ≈ 36; return to (40,90) @ 2950 RPM
 *   7. Gate        — sweep gate area (third pass); return to (52,90) @ 3050 RPM
 *   8. Gate        — sweep gate area (fourth pass); return to (52,90) @ 3050 RPM
 *   → Park
 *
 * Two gate-going paths: toGate (from 40,90) for cycles 5 & 7; toGateFar (from 52,90)
 * for cycles 3 & 8. toShoot1 and toShootFromGate both end at (52,90) so the robot
 * shoots there without any extra transit.
 * Blue is the primary coordinate frame; red is mirrored across x = 72.
 */
@Autonomous(name = "CloseAuto24", group = "Close")
public class CloseAuto24 extends CommandOpMode {

    Robot negabot;
    boolean isBlue = true;

    // ── Shooting-station tuning ─────────────────────────────────────────────
    // Near spot (40,90): preload, top row, bottom row  @ SHOOT_VELOCITY.
    // Far spot  (52,90): middle row + all gate cycles  @ SHOOT_VELOCITY_FAR.
    // Return paths already pass through (52,90), so far shots cost zero extra travel.
    static final double SHOOT_POS_X_INIT        = 58.81922196796339;
    static final double SHOOT_POS_Y_INIT        = 86.99313501144161;
    static final double SHOOT_HEADING_DEG_INIT  = 138;
    static final double SHOOT_POS_X        = 54.0672708+2;
    static final double SHOOT_POS_X_FAR    = 54.0672708+2;
    static final double SHOOT_POS_Y        = 71.57207864719906+5;
    static final double SHOOT_HEADING_DEG  = 46;
    static final double SHOOT_VELOCITY    = 2950.0;
    static final double SHOOT_VELOCITY_FAR = SHOOT_VELOCITY + 100.0;
    static final double SHOOT_VELOCITY_PRELOAD = SHOOT_VELOCITY + 300;
    static final double SHOOT_HOOD         = 0.5;
    static final double SHOOT_TURRET_OFFSET_DEG = 0.0;

    // ── Init / push-to-start ────────────────────────────────────────────────
    static final double INIT_X           = 48;
    static final double INIT_Y           = 120;
    static final double INIT_HEADING_DEG = 180;

    // ── Spike-row intake reach ──────────────────────────────────────────────
    // X the robot drives to when sweeping the artifact rows (middle/top/bottom).
    static final double spikeXintake = 10;

    // ── Gate-intake-station tuning ──────────────────────────────────────────
    static final double GATE_POS_X       = 11;
    static final double GATE_POS_Y       = 62.4-2;
    static final double GATE_HEADING_DEG = -15;
    static final double GATE_APPROACH_X  = 13;

    // ── Magazine cover positions ─────────────────────────────────────────────
    static final double COVER_OPEN  = .75;
    static final double COVER_CLOSE = .5;

    // ── Intake direction ─────────────────────────────────────────────────────
    static final double INTAKE_ON = -1.0;

    // ── Timing ──────────────────────────────────────────────────────────────
    static final long BURST_MS     = 550;   // feed time — was 750; matched to Zayan's ~500
    static final long GATE_WAIT_MS = 1000;

    // ── Path declarations ────────────────────────────────────────────────────
    // toGate: from (40,90) — cycles 5 & 7.
    // toGateFar: from (52,90) — cycles 3 & 8.
    // toShoot1 and toShootFromGate both end at (52,90).
    PathChain toShoot0;
    PathChain toMiddle,   toShoot1;
    PathChain toGate,     toGateFar,    toShootFromGate;
    PathChain toGateFarStraight, toShootFromGateStraight;   // straight versions once top row is gone
    PathChain toTopSweep, toShootFromTop;
    PathChain toBottom,   toShootFromBottom;
    PathChain park;

    @Override
    public void initialize() {
        // ── Alliance selection ────────────────────────────────────────────
        negabot = new Robot(hardwareMap, telemetry, initPose());
        negabot.shooter.setMagazineCover(COVER_CLOSE);
        Robot.Alliance shown = Robot.Alliance.BLUE;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            Robot.Alliance sel = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
            if (sel != shown) {
                negabot.drive.follower.setPose(initPose());
                shown = sel;
            }
            negabot.drive.follower.update();
            Pose live = negabot.drive.follower.getPose();
            telemetry.addLine("=== SELECT ALLIANCE (X/B), THEN PUSH TO START ===");
            telemetry.addData("Selected", isBlue ? ">>> BLUE <<<" : ">>>  RED <<<");
            telemetry.addData("Live pose", String.format("x=%.1f y=%.1f h=%.0f",
                    live.getX(), live.getY(), Math.toDegrees(live.getHeading())));
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        // ── Initialization ────────────────────────────────────────────────
        Robot.ALLIANCE = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
        negabot.turret.resetEncoder();

        Follower f = negabot.drive.follower;
        buildPaths(f);

        //final double shootTurret = shootTurretDeg();
        final double shootTurret = 30;

        // double[] lets the lambda read the current value on every tick.
        final double[] shootVel = { SHOOT_VELOCITY };

        // ── Default commands ──────────────────────────────────────────────
        negabot.shooter.setDefaultCommand(new CommandBase() {
            { addRequirements(negabot.shooter); }
            @Override public void execute()       { negabot.shooter.setVelocity(shootVel[0]); }
            @Override public boolean isFinished() { return false; }
        });

        negabot.turret.setDefaultCommand(new CommandBase() {
            { addRequirements(negabot.turret); }
            @Override public void execute()       { negabot.turret.setTargetDeg(shootTurret); }
            @Override public boolean isFinished() { return false; }
        });

        // ── Command schedule ──────────────────────────────────────────────
        negabot.schedule(
                new InstantCommand(() -> negabot.shooter.setHood(SHOOT_HOOD)),   // normal hood for the preload too

                new SequentialCommandGroup(

                        // ── 1: Preload — drive to the shoot spot, then shoot normally ──
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShoot0),   // drive to the spot while auto-aiming the turret
                        burst(),                 // stop, then fire like every other cycle

                        // ── 2: Middle row (y≈60) — returns to (52,90) @ 3050 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toMiddle, false),   // flow into the return, don't brake/hold
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> shootVel[0] = SHOOT_VELOCITY_FAR),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShoot1),
                        burst(),

                        // ── 3: Gate first pass — from (52,90), return to (52,90) @ 3050 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGateFar, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGate),
                        burst(),

                        // ── 5: Gate second pass — from (40,90), return to (52,90) @ 3050 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> shootVel[0] = SHOOT_VELOCITY_FAR),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGateStraight),
                        burst(),

                        // ── 6: Bottom row (y≈36) — from (52,90), returns to (40,90) @ 2950 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toBottom, false),   // flow into the return, don't brake/hold
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> shootVel[0] = SHOOT_VELOCITY),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromBottom),
                        burst(),

                        // ── 7: Gate third pass — from (40,90), return to (52,90) @ 3050 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> shootVel[0] = SHOOT_VELOCITY_FAR),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGateStraight),
                        burst(),


                        // ── 4: Top row (y≈84) — from (52,90), returns to (40,90) @ 2950 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toTopSweep, false),   // flow into the return, don't brake/hold
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> shootVel[0] = SHOOT_VELOCITY),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromTop),
                        burst(),

                        // ── Park — from (52,90) ───────────────────────────────────
                        new FollowPathCommand(f, park, true)
                )
        );
    }

    Command burst() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new WaitCommand(BURST_MS),
                new InstantCommand(() -> negabot.intake.setSpeed(0)),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    Command shotPrep(Follower f, PathChain path) {
        // Drive to the shoot spot while the turret continuously aims at the goal from
        // the live pose — so the chassis can be at ANY heading and the turret is
        // already on-target (pre-aimed during the drive) by the time we arrive.
        return new ParallelDeadlineGroup(
                new FollowPathCommand(f, path, true),
                new RunCommand(this::aimTurretAtGoal, negabot.turret)
        );
    }

    /** Aim the turret at the alliance goal from the robot's live pose (works at any heading). */
    void aimTurretAtGoal() {
        Pose p = negabot.drive.follower.getPose();
        if (p == null) return;
        double h = p.getHeading();
        double tx = p.getX() + 3 * Math.cos(h) - 3 * Math.sin(h);
        double ty = p.getY() + 0 * Math.sin(h) + 0 * Math.cos(h);
        Pose goal;
        if (isBlue)
        {
            goal = new Pose(FieldConstants.BLUE_GOAL_X, FieldConstants.BLUE_GOAL_Y);
        }
        else {
            goal = new Pose(FieldConstants.RED_GOAL_X, FieldConstants.RED_GOAL_Y);
        }

        double bearingDeg    = Math.toDegrees(Math.atan2(goal.getY() - ty, goal.getX() - tx));
        double deflectionDeg = wrap180(bearingDeg - Math.toDegrees(h));
        negabot.turret.setTargetDeg(135.0 - deflectionDeg + SHOOT_TURRET_OFFSET_DEG);
    }

    void buildPaths(Follower f) {

        Pose pushedStart = f.getPose();

        // ── Start → near shoot spot ──────────────────────────────────────────
        toShoot0 = f.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pushedStart.getX(), pushedStart.getY()),
                        sp(SHOOT_POS_X_INIT, SHOOT_POS_Y_INIT)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(SHOOT_HEADING_DEG_INIT))
                .build();

        // ── Middle row (y ≈ 60) ──────────────────────────────────────────────
        // 4-pt cubic: (40,90)→(52,90)→(52,60)→(36,60). Crosses y=84 at x≈47,
        // clearing the top-row artifact zone (x=18–36).
        toMiddle = f.pathBuilder()
                .addPath(new BezierLine(
                        sp(SHOOT_POS_X_INIT,     SHOOT_POS_Y_INIT),
                        sp(40.940503432494275,            60.17391304347828)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG_INIT), hr(0))
                .addPath(new BezierLine(sp(36.0, 57), sp(16, 57)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // 3-pt return: (18,60)→ctrl(52,60)→(52,90). Ends at far shoot spot.
        toShoot1 = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(16,    60.0),
                        sp(SHOOT_POS_X_FAR, 60.0),
                        sp(SHOOT_POS_X_FAR, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(SHOOT_HEADING_DEG))
                .build();

        // ── Gate (y ≈ 61) ───────────────────────────────────────────────────
        // toGate: 4-pt cubic from (40,90). Crosses y=84 at x≈47. Used cycles 5 & 7.
        // Straight: top row is already collected by cycles 5 & 7, so go direct.
        toGate = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(GATE_APPROACH_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(GATE_HEADING_DEG))
                .addPath(new BezierLine(sp(GATE_APPROACH_X, GATE_POS_Y), sp(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_HEADING_DEG))
                .build();

        // toGateFar: 3-pt quadratic from (52,90). Drops straight down then sweeps left.
        // Crosses y=84 at x≈52 — clear of artifacts. Used cycles 3 & 8.
        toGateFar = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(SHOOT_POS_X_FAR, SHOOT_POS_Y),
                        sp(SHOOT_POS_X_FAR, GATE_POS_Y),
                        sp(GATE_APPROACH_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(GATE_HEADING_DEG))
                .addPath(new BezierLine(sp(GATE_APPROACH_X, GATE_POS_Y), sp(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_HEADING_DEG))
                .build();

        // 3-pt return: (9.5,61)→ctrl(52,61)→(52,90). Ends at far shoot spot.
        toShootFromGate = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(GATE_POS_X,      GATE_POS_Y),
                        sp(SHOOT_POS_X_FAR, GATE_POS_Y),
                        sp(SHOOT_POS_X_FAR, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(SHOOT_HEADING_DEG))
                .build();

        // ── Straight gate paths — used by cycles 5, 7, 8 (top row already gone) ──
        toGateFarStraight = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X_FAR, SHOOT_POS_Y), sp(GATE_APPROACH_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(GATE_HEADING_DEG))
                .addPath(new BezierLine(sp(GATE_APPROACH_X, GATE_POS_Y), sp(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_HEADING_DEG))
                .build();

        toShootFromGateStraight = f.pathBuilder()
                .addPath(new BezierLine(sp(GATE_POS_X, GATE_POS_Y), sp(SHOOT_POS_X_FAR, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(SHOOT_HEADING_DEG))
                .build();

        // ── Top row (y ≈ 84) — starts from (52,90) after gate cycle 3 ────────
        // Control point (42,84) keeps path above y=84 until endpoint (36,84).
        toTopSweep = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X_FAR, SHOOT_POS_Y), sp(36.0, 84.0)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
                .addPath(new BezierLine(sp(36.0, 84.0), sp(spikeXintake+11, 84.0)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        toShootFromTop = f.pathBuilder()
                .addPath(new BezierLine(sp(spikeXintake, 84.0), sp(40.91533180778032, 127.08466819221968)))
                .setLinearHeadingInterpolation(hr(65), hr(65))
                .build();

        // ── Bottom row (y ≈ 36) — starts from (52,90) after gate cycle 5 ─────
        toBottom = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X_FAR, SHOOT_POS_Y), sp(36.0, 36.0)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
                .addPath(new BezierLine(sp(36.0, 36.0), sp(spikeXintake, 36.0)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        toShootFromBottom = f.pathBuilder()
                .addPath(new BezierLine(sp(spikeXintake, 36.0), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(SHOOT_HEADING_DEG))
                .build();

        // ── Park — from (52,90) after last gate shot ──────────────────────────
        park = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X_FAR, SHOOT_POS_Y), sp(47.0, 75.2)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(SHOOT_HEADING_DEG))
                .build();
    }

    @Override
    public void run() {
        if (negabot == null) return;
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }

    // ── Alliance-aware pose/heading helpers ─────────────────────────────────

    private Pose sp(double x, double y) {
        return isBlue ? new Pose(x, y) : new Pose(mx(x), y);
    }

    private Pose initPose() {
        return isBlue ? new Pose(INIT_X, INIT_Y, hr(INIT_HEADING_DEG))
                : new Pose(mx(INIT_X), INIT_Y, hr(INIT_HEADING_DEG));
    }

    private double hr(double deg) {
        return isBlue ? Math.toRadians(deg) : Math.toRadians(mhd(deg));
    }

    /**
     * Computes the turret angle (deg) that points at the alliance goal from the
     * shoot pose. Matches TurretTracking's formula exactly.
     */


    private static double wrap180(double a) {
        a = ((a + 180.0) % 360.0 + 360.0) % 360.0;
        return a - 180.0;
    }

    private static double mx(double x)    { return 144.0 - x; }
    private static double mhd(double deg) { return ((180.0 - deg) % 360.0 + 360.0) % 360.0; }
}
