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
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.PP.FieldConstants;

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
    static final double SHOOT_POS_X       = 52;    // shoot spot x (inches, blue) — +x to clear the top ball line on to/from paths
    static final double SHOOT_POS_Y       = 94;    // shoot spot y (inches)
    static final double SHOOT_HEADING_DEG = 30.0;    // robot heading at shoot spot (blue) — turret auto-aims, so tune this for a smooth gate transition (keep >= ~20°)
    static final double SHOOT_VELOCITY    = 3400.0;  // flywheel target velocity (RPM)
    static final double SHOOT_HOOD        = 0.15;    // hood servo position for shooting (0–1)
    // Turret now auto-aims at the goal from the shoot pose (see shootTurretDeg()).
    // This is just a fine correction added on top (deg, + = CW) if shots drift sideways.
    static final double SHOOT_TURRET_OFFSET_DEG = 0.0;

    // ── Init / push-to-start ────────────────────────────────────────────────
    // Robot is initialized here, then PUSHED to the real start during INIT;
    // odometry tracks the push and toShoot0 is built from the live pose.
    // INIT_HEADING_DEG MUST match the heading you physically place it at.
    static final double INIT_X           = 48.0;
    static final double INIT_Y           = 120.0;
    static final double INIT_HEADING_DEG = 90.0;

    // ── Gate-intake-station tuning ──────────────────────────────────────────
    // Robot returns to the shoot spot to fire after each gate cycle,
    // so GATE_HOOD matches SHOOT_HOOD (same shooting distance).
    static final double GATE_POS_X       = 9.5;     // gate intake x (inches, blue)
    static final double GATE_POS_Y       = 61.0;    // gate intake y (inches) — 2" less y
    static final double GATE_HEADING_DEG = 330.0;   // robot heading at the gate (blue)
    static final double GATE_APPROACH_X  = 14.0;    // x to reach before driving straight in along -x (also -6)
    static final double GATE_HOOD        = 0.15;    // hood servo position after gate cycle

    // ── Magazine cover positions ────────────────────────────────────────────
    static final double COVER_OPEN  = 1.0;   // swapped: servo was inverted
    static final double COVER_CLOSE = 0.1;

    // Intake runs continuously in this direction all auto (blocker gates the shooter).
    // Flip the sign if it runs the wrong way.
    static final double INTAKE_ON = -1.0;

    // ── Burst timing ───────────────────────────────────────────────────────
    // Intake runs backward for BURST_MS to fire all loaded artifacts (~3 × 500 ms each).
    static final long BURST_MS = 1000;

    // Let the blocker servo physically reach OPEN before the intake feeds.
    static final long COVER_SETTLE_MS = 250;

    // Time spent sitting at the gate to collect artifacts before returning to shoot.
    static final long GATE_WAIT_MS = 2000;

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
        // Created up front so the Pinpoint tracks the push during INIT.
        negabot = new Robot(hardwareMap, telemetry, initPose());
        negabot.shooter.setMagazineCover(COVER_CLOSE);   // blocker closed during init/push
        Robot.Alliance shown = Robot.Alliance.BLUE;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            Robot.Alliance sel = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
            if (sel != shown) {                              // alliance changed
                negabot.drive.follower.setPose(initPose());  // re-zero init pose
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

        // ── Robot initialization ──────────────────────────────────────────
        Robot.ALLIANCE = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;

        negabot.turret.resetEncoder();

        Follower f = negabot.drive.follower;
        buildPaths(f);

        double shootTurret = shootTurretDeg();

        // ── Command schedule ──────────────────────────────────────────────
        negabot.schedule(
            new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE)),
            new InstantCommand(() -> negabot.shooter.setHoodPosition(SHOOT_HOOD)),
            new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),   // intake always on

            new SequentialCommandGroup(

                // ── 1: Preload — drive to shoot spot and fire ─────────────
                shotPrep(f, toShoot0, shootTurret),
                burst(),

                // ── 2: Middle row (y ≈ 62) ───────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toMiddleA, true),
                new FollowPathCommand(f, toMiddleB, true),
                shotPrep(f, toShoot1, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── 3: Gate (first pass) ──────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                new WaitCommand(GATE_WAIT_MS),
                shotPrep(f, toShootFromGate, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── 4: Top row (y ≈ 84) ──────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toTopSweep, true),
                shotPrep(f, toShootFromTop, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── 5: Gate (second pass) ─────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                new WaitCommand(GATE_WAIT_MS),
                shotPrep(f, toShootFromGate, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── 6: Bottom row (y ≈ 37) ───────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toBottomA, true),
                new FollowPathCommand(f, toBottomB, true),
                shotPrep(f, toShootFromBottom, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── 7: Gate (third pass) ──────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                new WaitCommand(GATE_WAIT_MS),
                shotPrep(f, toShootFromGate, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── 8: Gate (fourth pass) ─────────────────────────────────
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new FollowPathCommand(f, toGateApproach, true),
                new FollowPathCommand(f, toGateSweep, true),
                new WaitCommand(GATE_WAIT_MS),
                shotPrep(f, toShootFromGate, shootTurret),
                //new WaitCommand(100),
                burst(),

                // ── Park ──────────────────────────────────────────────────
                new FollowPathCommand(f, park, true)
            )
        );
    }

    // ── burst: open blocker, fire all loaded artifacts, close blocker ───────
    // The blocker is opened ONLY here (right before feeding) and closed right
    // after — shotPrep no longer touches it.
    Command burst() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
            new WaitCommand(COVER_SETTLE_MS),   // open the blocker; the always-on intake feeds
            new WaitCommand(BURST_MS),
            new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    // ── shotPrep: drive to shoot spot and aim the turret ────────────────────
    // Hood + velocity are set once at start and never touched; the blocker stays
    // closed through the whole approach and is opened by burst() right before
    // firing — so there is nothing else to prep here.
    Command shotPrep(Follower f, PathChain path, double turretDeg) {
        return new ParallelCommandGroup(
            new FollowPathCommand(f, path, true),
            new InstantCommand(() -> negabot.turret.setTargetDeg(turretDeg))
        );
    }

    // ── buildPaths ───────────────────────────────────────────────────────────
    void buildPaths(Follower f) {

        // ── 1: Preload — start position → shoot spot ────────────────────────
        Pose pushedStart = f.getPose();   // actual start after the push
        toShoot0 = f.pathBuilder()
            .addPath(new BezierLine(
                new Pose(pushedStart.getX(), pushedStart.getY()),
                sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(SHOOT_HEADING_DEG))
            .build();

        // ── 2: Middle row (y ≈ 62) — approach diagonal, then sweep left ─────
        toMiddleA = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(36.0, 60.0)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
            .build();

        toMiddleB = f.pathBuilder()
            .addPath(new BezierLine(sp(36.0, 60.0), sp(18.0, 60.0)))
            .setLinearHeadingInterpolation(hr(0), hr(0))
            .build();

        toShoot1 = f.pathBuilder()
            .addPath(new BezierLine(sp(18.0, 60.0), sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(0), hr(SHOOT_HEADING_DEG))
            .build();

        // ── Gate paths — shared across cycles 3, 5, 7, 8 ───────────────────
        // Approach: shoot spot → gate entry, ending already at the gate heading
        toGateApproach = f.pathBuilder()
            .addPath(new BezierLine(
                sp(SHOOT_POS_X, SHOOT_POS_Y),
                sp(GATE_APPROACH_X, GATE_POS_Y)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(GATE_HEADING_DEG))
            .build();

        // Drive straight into the gate parallel to the x-axis (heading held)
        toGateSweep = f.pathBuilder()
            .addPath(new BezierLine(
                sp(GATE_APPROACH_X, GATE_POS_Y),
                sp(GATE_POS_X, GATE_POS_Y)))
            .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_HEADING_DEG))
            .build();

        // Return: straight back to shoot spot
        toShootFromGate = f.pathBuilder()
            .addPath(new BezierLine(
                sp(GATE_POS_X, GATE_POS_Y),
                sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(SHOOT_HEADING_DEG))
            .build();

        // ── 4: Top row (y ≈ 84) — single lateral sweep from shoot spot ──────
        // The shoot spot is at y=85, so this sweeps the adjacent row in one pass.
        toTopSweep = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(36.0, 84.0)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
            .addPath(new BezierLine(sp(36.0, 84.0), sp(18.0, 84.0)))
            .setLinearHeadingInterpolation(hr(0), hr(0))
            .build();

        toShootFromTop = f.pathBuilder()
            .addPath(new BezierLine(sp(18.0, 84.0), sp(SHOOT_POS_X, SHOOT_POS_Y)))
            .setLinearHeadingInterpolation(hr(0), hr(SHOOT_HEADING_DEG))
            .build();

        // ── 6: Bottom row (y ≈ 37) — approach diagonal, then sweep left ─────
        toBottomA = f.pathBuilder()
            .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(36.0, 36.0)))
            .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(0))
            .build();

        toBottomB = f.pathBuilder()
            .addPath(new BezierLine(sp(36.0, 36.0), sp(18.0, 36.0)))
            .setLinearHeadingInterpolation(hr(0), hr(0))
            .build();

        toShootFromBottom = f.pathBuilder()
            .addPath(new BezierLine(sp(18.0, 36.0), sp(SHOOT_POS_X, SHOOT_POS_Y)))
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

    /** Init pose the robot is placed at before being pushed (mirrored for red). */
    private Pose initPose() {
        return isBlue ? new Pose(INIT_X, INIT_Y, hr(INIT_HEADING_DEG))
                      : new Pose(mx(INIT_X), INIT_Y, hr(INIT_HEADING_DEG));
    }

    /** Heading in radians, mirrored for red. */
    private double hr(double deg) {
        return isBlue ? Math.toRadians(deg) : Math.toRadians(mhd(deg));
    }

    /**
     * Turret angle that aims at the alliance goal from the shoot spot, for the
     * current SHOOT_HEADING_DEG. Because this is computed (not a fixed constant),
     * the robot heading can be chosen freely (e.g. to flow into the gate) and the
     * turret still hits. Same convention as PPTracking: turret = 135 − deflection.
     */
    double shootTurretDeg() {
        Pose shoot = sp(SHOOT_POS_X, SHOOT_POS_Y);
        double hRad = hr(SHOOT_HEADING_DEG);
        double tx = shoot.getX() + Turret.FORWARD_OFFSET_IN * Math.cos(hRad);
        double ty = shoot.getY() + Turret.FORWARD_OFFSET_IN * Math.sin(hRad);
        Pose goal = FieldConstants.goalAimPointForAlliance(isBlue);
        double bearingDeg    = Math.toDegrees(Math.atan2(goal.getY() - ty, goal.getX() - tx));
        double deflectionDeg = wrap180(bearingDeg - Math.toDegrees(hRad));
        return Turret.clampDeg(135.0 - deflectionDeg + SHOOT_TURRET_OFFSET_DEG);
    }

    /** Wrap an angle to (−180, 180]. */
    private static double wrap180(double a) {
        a = ((a + 180.0) % 360.0 + 360.0) % 360.0;
        return a - 180.0;
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
