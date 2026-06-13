package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

/**
 * 24-Artifact Close Autonomous — DECODE 2025-2026.
 *
 * Select alliance with gamepad1 before pressing START:
 *   X  →  BLUE  (default)
 *   B  →  RED
 *
 * Cycle order (8 total):
 *   1. Preload     — fire pre-loaded artifacts at the shoot spot
 *   2. Middle row  — sweep row at y ≈ 62
 *   3. Gate        — sweep gate area
 *   4. Top row     — sweep row at y ≈ 84
 *   5. Gate        — sweep gate area (second pass)
 *   6. Bottom row  — sweep row at y ≈ 37
 *   7. Gate        — sweep gate area (third pass)
 *   8. Gate        — sweep gate area (fourth pass)
 *   → Park
 *
 * Gate paths are shared objects reused across all four gate cycles.
 * Blue is the primary coordinate frame; red is mirrored across x = 72.
 */
@Autonomous(name = "CloseAuto24", group = "Close")
public class CloseAuto24 extends CommandOpMode {

    Robot negabot;
    boolean isBlue = true;

    // ── Shooting-station tuning ─────────────────────────────────────────────
    static final double SHOOT_POS_X       = 58.5;    // shoot spot x (inches, blue)
    static final double SHOOT_POS_Y       = 85.0;    // shoot spot y (inches)
    static final double SHOOT_HEADING_DEG = 54.0;    // robot heading at shoot spot (blue)
    static final double SHOOT_VELOCITY    = 3400.0;  // flywheel target velocity (RPM)
    static final double SHOOT_HOOD        = 0.15;    // hood servo position for shooting (0–1)
    static final double SHOOT_TURRET_DEG  = 50.0;    // turret angle at shoot spot (blue, degrees)

    // ── Gate-intake-station tuning ──────────────────────────────────────────
    // Robot returns to the shoot spot to fire after each gate cycle,
    // so GATE_HOOD matches SHOOT_HOOD (same shooting distance).
    static final double GATE_POS_X       = 14.0;    // gate area approach x (inches, blue)
    static final double GATE_POS_Y       = 65.0;    // gate area approach y (inches)
    static final double GATE_HEADING_DEG = -20.0;   // robot heading while sweeping gate (blue)
    static final double GATE_HOOD        = 0.15;    // hood servo position after gate cycle

    // ── Magazine cover positions ────────────────────────────────────────────
    static final double COVER_OPEN  = 0.1;
    static final double COVER_CLOSE = 1.0;

    // ── Burst timing ───────────────────────────────────────────────────────
    // Intake runs backward for BURST_MS to fire all loaded artifacts (~3 × 500 ms each).
    static final long BURST_MS = 1000;

    // ── Path declarations ───────────────────────────────────────────────────
    PathChain toShoot0;                                  // 1: start → shoot (preload)
    PathChain toMiddleA, toMiddleB, toShoot1;            // 2: middle row y≈62
    PathChain toGateApproach, toGateSweep, toShootFromGate; // 3,5,7,8: gate (reused 4×)
    PathChain toTopSweep, toShootFromTop;                // 4: top row y≈84
    PathChain toBottomA, toBottomB, toShootFromBottom;   // 6: bottom row y≈37
    PathChain park;

    @Override
    public void initialize() {
        // ── Alliance selection (gamepad, before START) ────────────────────
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            telemetry.addLine("=== CloseAuto24  SELECT ALLIANCE ===");
            telemetry.addLine("  X = BLUE   |   B = RED");
            telemetry.addData("Selected", isBlue ? ">>> BLUE <<<" : ">>>  RED <<<");
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        // ── Robot initialization ──────────────────────────────────────────
        Robot.ALLIANCE = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;

        Pose startPose = isBlue
                ? new Pose(26.241, 133.326, hr(SHOOT_HEADING_DEG))
                : new Pose(mx(26.241), 133.326, hr(SHOOT_HEADING_DEG));

        negabot = new Robot(hardwareMap, telemetry, startPose);
        negabot.turret.resetEncoder();

        Follower f = negabot.drive.follower;
        buildPaths(f);

        double shootTurret = isBlue ? SHOOT_TURRET_DEG : mt(SHOOT_TURRET_DEG);

        // ── Command schedule ──────────────────────────────────────────────
        negabot.schedule(
            new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE)),
            new InstantCommand(() -> negabot.shooter.setHoodPosition(SHOOT_HOOD)),

            new SequentialCommandGroup(

                // ── 1: Preload — drive to shoot spot and fire ─────────────
                shotPrep(f, toShoot0, shootTurret, SHOOT_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 2: Middle row (y ≈ 62) ───────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toMiddleA, true),
                new FollowPathCommand(f, toMiddleB, true),
                shotPrep(f, toShoot1, shootTurret, SHOOT_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 3: Gate (first pass) ──────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                shotPrep(f, toShootFromGate, shootTurret, GATE_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 4: Top row (y ≈ 84) ──────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toTopSweep, true),
                shotPrep(f, toShootFromTop, shootTurret, SHOOT_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 5: Gate (second pass) ─────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                shotPrep(f, toShootFromGate, shootTurret, GATE_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 6: Bottom row (y ≈ 37) ───────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toBottomA, true),
                new FollowPathCommand(f, toBottomB, true),
                shotPrep(f, toShootFromBottom, shootTurret, SHOOT_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 7: Gate (third pass) ──────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                shotPrep(f, toShootFromGate, shootTurret, GATE_HOOD),
                new WaitCommand(100),
                burst(),

                // ── 8: Gate (fourth pass) ─────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(1.0)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                shotPrep(f, toShootFromGate, shootTurret, GATE_HOOD),
                new WaitCommand(100),
                burst(),

                // ── Park ──────────────────────────────────────────────────
                new FollowPathCommand(f, park, true)
            )
        );
    }

    // ── burst: fire all loaded artifacts ────────────────────────────────────
    // Opens cover as a safety fallback (also opened mid-drive by shotPrep).
    Command burst() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
            new InstantCommand(() -> negabot.intake.setSpeed(-1.0)),
            new WaitCommand(BURST_MS),
            new InstantCommand(() -> negabot.intake.setSpeed(0.0)),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    // ── shotPrep: drive to shoot spot while aiming turret/hood/cover ────────
    // Cover opens 500 ms into the drive so it is ready on arrival.
    Command shotPrep(Follower f, PathChain path, double turretDeg, double hoodPos) {
        return new ParallelCommandGroup(
            new FollowPathCommand(f, path, true),
            new InstantCommand(() -> negabot.intake.setSpeed(0.0)),
            new InstantCommand(() -> negabot.turret.setTargetDeg(turretDeg)),
            new InstantCommand(() -> negabot.shooter.setHoodPosition(hoodPos)),
            new SequentialCommandGroup(
                new WaitCommand(500),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN))
            )
        );
    }

    // ── buildPaths ───────────────────────────────────────────────────────────
    void buildPaths(Follower f) {

        // ── 1: Preload — start position → shoot spot ────────────────────────
        toShoot0 = f.pathBuilder()
            .addPath(new BezierLine(sp(26.241, 133.326), sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(SHOOT_HEADING_DEG))
            .build();

        // ── 2: Middle row (y ≈ 62) — approach diagonal, then sweep left ─────
        toMiddleA = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(42.0, 67.5)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
            .build();

        toMiddleB = f.pathBuilder()
            .addPath(new BezierLine(sp(42.0, 62.5), sp(15.5, 66.5)))
            .setLinearHeadingInterpolation(hr(0), hr(0))
            .build();

        toShoot1 = f.pathBuilder()
            .addPath(new BezierCurve(
                sp(15.5, 61.5), sp(47.0, 68.0), sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(90), hr(SHOOT_HEADING_DEG))
            .build();

        // ── Gate paths — shared across cycles 3, 5, 7, 8 ───────────────────
        // Approach: shoot spot → gate entry point
        toGateApproach = f.pathBuilder()
            .addPath(new BezierLine(
                sp(SHOOT_POS_X, SHOOT_POS_Y),
                sp(GATE_POS_X + 6.0, GATE_POS_Y + 2.0)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(GATE_HEADING_DEG))
            .build();

        // Sweep: ~14 inches along the gate to collect ~3 artifacts
        toGateSweep = f.pathBuilder()
            .addPath(new BezierLine(
                sp(GATE_POS_X + 6.0, GATE_POS_Y + 2.0),
                sp(GATE_POS_X - 7.0, GATE_POS_Y + 8.0)))
            .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_HEADING_DEG))
            .build();

        // Return: curve back to shoot spot
        toShootFromGate = f.pathBuilder()
            .addPath(new BezierCurve(
                sp(GATE_POS_X - 7.0, GATE_POS_Y + 8.0),
                sp(35.0, 72.0),
                sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(SHOOT_HEADING_DEG))
            .build();

        // ── 4: Top row (y ≈ 84) — single lateral sweep from shoot spot ──────
        // The shoot spot is at y=85, so this sweeps the adjacent row in one pass.
        toTopSweep = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(14.0, 84.0)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
            .build();

        toShootFromTop = f.pathBuilder()
            .addPath(new BezierLine(sp(14.0, 84.0), sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(0), hr(SHOOT_HEADING_DEG))
            .build();

        // ── 6: Bottom row (y ≈ 37) — approach diagonal, then sweep left ─────
        toBottomA = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(41.0, 37.9)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
            .build();

        toBottomB = f.pathBuilder()
            .addPath(new BezierLine(sp(41.0, 37.9), sp(14.7, 37.4)))
            .setLinearHeadingInterpolation(hr(0), hr(0))
            .build();

        toShootFromBottom = f.pathBuilder()
            .addPath(new BezierLine(sp(14.7, 37.4), sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(0), hr(SHOOT_HEADING_DEG))
            .build();

        // ── Park ─────────────────────────────────────────────────────────────
        park = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(47.0, 75.2)))
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

    /** Field pose: blue coordinates direct, red x-mirrored. */
    private Pose sp(double x, double y) {
        return isBlue ? new Pose(x, y) : new Pose(mx(x), y);
    }

    /** Heading in radians, mirrored for red. */
    private double hr(double deg) {
        return isBlue ? Math.toRadians(deg) : Math.toRadians(mhd(deg));
    }

    // ── Mirror math (same convention as RedSideClose.java) ──────────────────

    /** Mirror x across the field centre (x = 72). */
    private static double mx(double x) {
        return 144.0 - x;
    }

    /** Mirror a heading in degrees across the vertical axis: θ → (180 − θ). */
    private static double mhd(double deg) {
        return ((180.0 - deg) % 360.0 + 360.0) % 360.0;
    }

    /** Mirror a turret angle around the 135° forward axis: t → (270° − t). */
    private static double mt(double turretDeg) {
        return ((270.0 - turretDeg) % 360.0 + 360.0) % 360.0;
    }
}
