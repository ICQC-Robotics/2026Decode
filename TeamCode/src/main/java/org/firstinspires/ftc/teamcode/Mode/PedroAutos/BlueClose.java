package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PP.FieldConstants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.ClampedPPTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitUntilReadyToShoot;

/**
 * Blue-alliance-only split of ZayansFull24Auto -- fixed to BLUE, no alliance prompt, no
 * mirroring helpers (every position/heading/turret-angle constant below is already
 * blue-relative). Behavior is otherwise identical: preload, middle row, gate, gate, gate, bottom
 * (3rd) row, top row -- or, with bottom row disabled (toggleable at init, defaults ON), preload,
 * middle row, gate x4, top row. See RedClose for the mirrored counterpart.
 */
@Autonomous(name = "BlueClose")
public class BlueClose extends CommandOpMode {

    Robot negabot;
    boolean doBottomRow = true;

    // ── Shooting-station tuning ─────────────────────────────────────────────
    // Cycle order: preload, middle row, gate, gate, gate, bottom row, top row.
    static final double SHOOT_POS_X        = 54.0672708+2;
    static final double SHOOT_POS_Y        = 71.57207864719906+5;
    static final double SHOOT_HEADING_DEG  = 46;
    // The single base turret center, tuned for a robot heading of SHOOT_HEADING_DEG. Every other
    // shot's center is this same number shifted by however far ITS heading differs from that --
    // see turretCenterDeg(double) -- except the final (top row) shot, which gets its own
    // independent, hand-tunable constant: TOP_TURRET_DEG below.
    static final double DEFAULT_TURRET_DEG = 50.5;
    static final double TURRET_WINDOW      = 20;

    // Gate before every shot: don't fire until the chassis has settled and the turret is on
    // target, so the actual shot doesn't happen while still moving/rotating. Bounded by
    // SHOOT_PAUSE_MS so a noisy localizer or out-of-range turret target can't hang the routine.
    static final double MAX_SETTLE_VELOCITY    = 2;    // in/sec, "stopped" threshold
    static final double TURRET_ALIGN_TOLERANCE = 2.0;  // deg, "aligned" threshold
    static final long   SHOOT_PAUSE_MS         = 200;

    // toShootFromGate/toShootFromGateBack's heading interpolation only explicitly ramps over
    // t=[0.1,0.3] of the path, then falls back to plain tangent-following for the rest -- so the
    // robot actually arrives facing each line's tangent angle, not SHOOT_HEADING_DEG. Recompute
    // these (atan2 of the line's (dx, dy)) if GATE_POS, SHOOT_POS, or BACK_POS ever change.
    static final double GATE_ARRIVAL_HEADING_DEG      = 20.6739116972064316;
    static final double GATE_BACK_ARRIVAL_HEADING_DEG = 16.5429834045625341;

    // Kept independent of turretCenterDeg(...)/DEFAULT_TURRET_DEG -- a hardcoded literal, not
    // derived from it -- so the final shot of the auto can be hand-tuned on the field, or the
    // shared base retuned, without either affecting the other.
    static final double TOP_TURRET_DEG = 51.5;

    // The 3rd gate visit and the bottom row both shoot from here instead of the regular spot --
    // it's a shorter hop from the gate straight into the bottom row.
    static final double BACK_POS_X = 60;
    static final double BACK_POS_Y = 74;

    // Top row shoots from here instead of the regular spot.
    static final double TOP_POS_X = 55;
    static final double TOP_POS_Y = 105;

    // Same turret-forward offset PPTracking/ClampedPPTracking use (package-private over there),
    // duplicated here just to estimate each station's distance-to-goal for the AutoAim LUT.
    static final double TURRET_FORWARD_OFFSET_IN = 3;

    // ── RPM gate (preload only) ───────────────────────────────────────────
    static final double RPM_TOLERANCE       = 100;
    static final long   RPM_WAIT_TIMEOUT_MS = 1500;

    // ── Init / push-to-start ────────────────────────────────────────────────
    static final double INIT_X           = 48;
    static final double INIT_Y           = 120;
    static final double INIT_HEADING_DEG = 180;

    // ── Gate-intake-station tuning ──────────────────────────────────────────
    static final double GATE_POS_X       = 10.5; //Even: 10.5 Odd: Unknown
    static final double GATE_POS_Y       = 60.5; //Even: 60.5 Odd: Unknown
    static final double GATE_HEADING_DEG = 345;

    // ── Spike-row intake reach ──────────────────────────────────────────────
    static final double spikeXintake = 20;

    // ── Magazine cover positions ─────────────────────────────────────────────
    static final double COVER_OPEN  = .75;
    static final double COVER_CLOSE = .55;

    // ── Intake direction ─────────────────────────────────────────────────────
    static final double INTAKE_ON = -1.0;

    // ── Timing ──────────────────────────────────────────────────────────────
    static final long BURST_MS     = 525;
    static final long FAST_GATE_WAIT_MS = 625;

    // Low brakingStart -> the chain keeps cruising at full speed and only decelerates right at the
    // very end, instead of slowing down early -- used on paths that drive into an intake/row/gate.
    static final double INTAKE_BRAKING_START = 0.05;

    PathChain toShoot0;
    PathChain toMiddle,   toShoot1;
    PathChain toGate,     toShootFromGate, toShootFromGateBack;
    PathChain toTopSweep, toTopSweepFromRegular, toShootFromTop;
    PathChain toBottom,   toShootFromBottom;

    // Hood/velocity per shooting station, computed once from AutoAim's teleop-tuned distance LUT.
    double regularHood, regularVelocity;
    double backHood,    backVelocity;
    double topHood,     topVelocity;

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, initPose());
        negabot.shooter.setMagazineCover(COVER_CLOSE);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.y) doBottomRow = true;
            if (gamepad1.a) doBottomRow = false;
            negabot.drive.follower.update();
            Pose live = negabot.drive.follower.getPose();
            telemetry.addLine("=== BLUE CLOSE — PUSH TO START ===");
            telemetry.addData("Bottom Row", doBottomRow ? "ON  (A=disable)" : "OFF (Y=enable)");
            telemetry.addData("Live pose", String.format("x=%.1f y=%.1f h=%.0f",
                    live.getX(), live.getY(), Math.toDegrees(live.getHeading())));
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        // ── Initialization ────────────────────────────────────────────────
        Robot.ALLIANCE = Robot.Alliance.BLUE;
        negabot.turret.resetEncoder();
        computeShotParams();

        Follower f = negabot.drive.follower;
        buildPaths(f);

        // ── Command schedule ──────────────────────────────────────────────
        negabot.schedule(
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new InstantCommand(() -> negabot.turret.setTargetDeg(turretCenterDeg(SHOOT_HEADING_DEG) - 4)),
                new InstantCommand(() -> negabot.shooter.setVelocity(regularVelocity)),
                new InstantCommand(() -> negabot.shooter.setHood(regularHood)),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE)),
                new SequentialCommandGroup(
                        // ── Preload (cycle 1) — regular station ──
                        shotPrep(f, toShoot0, turretCenterDeg(SHOOT_HEADING_DEG) - 4, TURRET_WINDOW),
                        waitForRpm(),
                        readyToShoot(f),
                        burst(),

                        // ── Middle row (cycle 2) — regular station ──
                        prepNextShot(regularHood, regularVelocity, turretCenterDeg(SHOOT_HEADING_DEG)),
                        new FollowPathCommand(f, toMiddle, false),
                        shotPrep(f, toShoot1, turretCenterDeg(SHOOT_HEADING_DEG) - 2.5, TURRET_WINDOW),
                        readyToShoot(f),
                        burst(),

                        // ── Gate 1 (cycle 3) — regular station ──
                        prepNextShot(regularHood, regularVelocity, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(FAST_GATE_WAIT_MS - 150),
                        shotPrep(f, toShootFromGate, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG), TURRET_WINDOW),
                        readyToShoot(f),
                        burst(),

                        // ── Gate 2 (cycle 4) — regular station ──
                        prepNextShot(regularHood, regularVelocity, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(FAST_GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG), TURRET_WINDOW),
                        readyToShoot(f),
                        burst(),

                        // ── Bottom-row toggle ──
                        // ON:  gate 3 (-> BACK), bottom row (BACK <-> BACK), top row (BACK -> TOP).
                        // OFF: gate 3 and gate 4 both stay at the regular station; with no bottom
                        //      row, the robot never needs to visit BACK_POS at all, so top row
                        //      sweeps in from the regular station instead.
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        // ── Gate 3 (cycle 5) — returns to the BACK station ──
                                        prepNextShot(backHood, backVelocity, turretCenterDeg(GATE_BACK_ARRIVAL_HEADING_DEG)),
                                        new FollowPathCommand(f, toGate, true),
                                        new WaitCommand(FAST_GATE_WAIT_MS),
                                        shotPrep(f, toShootFromGateBack, turretCenterDeg(GATE_BACK_ARRIVAL_HEADING_DEG), TURRET_WINDOW),
                                        readyToShoot(f),
                                        burst(),

                                        // ── Bottom / 3rd row (cycle 6) — BACK station both ways ──
                                        prepNextShot(backHood, backVelocity, turretCenterDeg(SHOOT_HEADING_DEG)),
                                        new FollowPathCommand(f, toBottom, false),
                                        shotPrep(f, toShootFromBottom, turretCenterDeg(SHOOT_HEADING_DEG), TURRET_WINDOW),
                                        readyToShoot(f),
                                        burst(),

                                        // ── Top row (final shot) — TOP station, sweeping in from BACK ──
                                        prepNextShot(topHood, topVelocity, TOP_TURRET_DEG),
                                        new FollowPathCommand(f, toTopSweep, false),
                                        shotPrep(f, toShootFromTop, TOP_TURRET_DEG, TURRET_WINDOW),
                                        readyToShoot(f),
                                        finalBurst()
                                ),
                                new SequentialCommandGroup(
                                        // ── Gate 3 (cycle 5) — regular station ──
                                        prepNextShot(regularHood, regularVelocity, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG)),
                                        new FollowPathCommand(f, toGate, true),
                                        new WaitCommand(FAST_GATE_WAIT_MS),
                                        shotPrep(f, toShootFromGate, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG), TURRET_WINDOW),
                                        readyToShoot(f),
                                        burst(),

                                        // ── Gate 4 (cycle 6) — regular station, replaces bottom row ──
                                        prepNextShot(regularHood, regularVelocity, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG)),
                                        new FollowPathCommand(f, toGate, true),
                                        new WaitCommand(FAST_GATE_WAIT_MS),
                                        shotPrep(f, toShootFromGate, turretCenterDeg(GATE_ARRIVAL_HEADING_DEG), TURRET_WINDOW),
                                        readyToShoot(f),
                                        burst(),

                                        // ── Top row (final shot) — TOP station, sweeping in from regular ──
                                        prepNextShot(topHood, topVelocity, TOP_TURRET_DEG),
                                        new FollowPathCommand(f, toTopSweepFromRegular, false),
                                        shotPrep(f, toShootFromTop, TOP_TURRET_DEG, TURRET_WINDOW),
                                        readyToShoot(f),
                                        finalBurst()
                                ),
                                () -> doBottomRow
                        )
                )
        );
    }

    Command burst() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                new WaitCommand(BURST_MS),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    // Same as burst(), but leaves the blocker open afterward instead of closing it -- only used
    // for the last shot of the auto, since there's nothing left afterward to protect against.
    Command finalBurst() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                new WaitCommand(BURST_MS)
        );
    }

    // Blocks (up to SHOOT_PAUSE_MS) until the chassis has stopped and the turret is on target, so
    // a shot doesn't fire while still moving/rotating.
    Command readyToShoot(Follower f) {
        return new WaitUntilReadyToShoot(f, negabot.turret, MAX_SETTLE_VELOCITY, TURRET_ALIGN_TOLERANCE, SHOOT_PAUSE_MS);
    }

    // Blocks (up to RPM_WAIT_TIMEOUT_MS) until the shooter is within RPM_TOLERANCE of target velocity.
    Command waitForRpm() {
        return new ParallelRaceGroup(
                new CommandBase() {
                    @Override
                    public boolean isFinished() {
                        return negabot.shooter.getVelocity() >= negabot.shooter.getTargetVelocity() - RPM_TOLERANCE;
                    }
                },
                new WaitCommand(RPM_WAIT_TIMEOUT_MS)
        );
    }

    /**
     * Starts the turret/hood/shooter moving toward the next shot's values right away, instead of
     * waiting until the robot arrives -- gives them the whole next leg to settle.
     */
    Command prepNextShot(double hood, double velocity, double resolvedTurretCenterDeg) {
        return new InstantCommand(() -> {
            negabot.shooter.setHood(hood);
            negabot.shooter.setVelocity(velocity);
            negabot.turret.setTargetDeg(resolvedTurretCenterDeg);
        });
    }

    public Command shotPrep(Follower f, PathChain path, double centerDeg, double windowDeg) {
        return new ParallelDeadlineGroup(
                new FollowPathCommand(f, path, true),
                new ClampedPPTracking(
                        negabot.turret,
                        negabot.drive,
                        Robot.ALLIANCE,
                        centerDeg - windowDeg,
                        centerDeg + windowDeg
                )
        );
    }

    /**
     * Looks up hood/velocity for each shooting station from AutoAim's teleop-tuned distance LUT
     * instead of hand-picked constants.
     */
    void computeShotParams() {
        double regularDist = distanceToGoalIn(SHOOT_POS_X, SHOOT_POS_Y, SHOOT_HEADING_DEG);
        double backDist    = distanceToGoalIn(BACK_POS_X, BACK_POS_Y, SHOOT_HEADING_DEG);
        double topDist     = distanceToGoalIn(TOP_POS_X, TOP_POS_Y, SHOOT_HEADING_DEG);

        regularHood     = AutoAim.getHoodForDistance(regularDist);
        regularVelocity = AutoAim.getRpmForDistance(regularDist);
        backHood        = AutoAim.getHoodForDistance(backDist);
        backVelocity    = AutoAim.getRpmForDistance(backDist);
        topHood         = AutoAim.getHoodForDistance(topDist);
        topVelocity     = AutoAim.getRpmForDistance(topDist);
    }

    private double distanceToGoalIn(double x, double y, double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        double turretX = x + TURRET_FORWARD_OFFSET_IN * Math.cos(headingRad);
        double turretY = y + TURRET_FORWARD_OFFSET_IN * Math.sin(headingRad);
        return Math.hypot(FieldConstants.BLUE_GOAL_X - turretX, FieldConstants.BLUE_GOAL_Y - turretY);
    }

    void buildPaths(Follower f) {

        Pose pushedStart = f.getPose();

        toShoot0 = f.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pushedStart.getX(), pushedStart.getY()),
                        new Pose(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), Math.toRadians(SHOOT_HEADING_DEG))
                .build();

        toGate = f.pathBuilder()
                .addPath(new BezierLine(new Pose(SHOOT_POS_X, SHOOT_POS_Y), new Pose(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(GATE_HEADING_DEG))
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        toShootFromGate = f.pathBuilder()
                .addPath(new BezierLine(new Pose(GATE_POS_X, GATE_POS_Y), new Pose(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(GATE_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG), 0.3, 0.1)
                .build();

        // Last gate visit returns to the BACK station instead -- a shorter hop into bottom row.
        toShootFromGateBack = f.pathBuilder()
                .addPath(new BezierLine(new Pose(GATE_POS_X, GATE_POS_Y), new Pose(BACK_POS_X, BACK_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(GATE_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG), 0.3, 0.1)
                .build();

        // ── Middle row (y ≈ 60) — curved sweep, mirrors CloseAuto18Leave's row-curve pattern ──
        toMiddle = f.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(SHOOT_POS_X, SHOOT_POS_Y),
                        new Pose(45, 60),
                        new Pose(22, 60)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        toShoot1 = f.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(22, 60),
                        new Pose(45, 60),
                        new Pose(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();

        // ── 3rd/bottom row (y ≈ 36) — starts AND ends at the BACK station, since gate 3 just
        // dropped the robot off there and top row picks up from there too ─────────────────────
        toBottom = f.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(BACK_POS_X, BACK_POS_Y),
                        new Pose(45, 36.0),
                        new Pose(spikeXintake, 36.0)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        toShootFromBottom = f.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(spikeXintake, 36.0),
                        new Pose(45, 36.0),
                        new Pose(BACK_POS_X, BACK_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();

        // Single straight line from the BACK station to (22, 84). True tangent/reversed would
        // snap instantly from the incoming 46° (the "wild swing"), so the first 30% (in time, not
        // distance) ramps heading linearly to the path's back-leading angle; the remaining 70%
        // follows that same line tangentially/reversed. -14.744...° is this specific line's
        // reversed tangent at the endpoint -- recompute it if BACK_POS or this endpoint changes.
        toTopSweep = f.pathBuilder()
                .addPath(new BezierLine(new Pose(BACK_POS_X, BACK_POS_Y), new Pose(22, 84)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, 0.3,
                                HeadingInterpolator.linear(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(-14.7435628364706872), 0.3)),
                        new HeadingInterpolator.PiecewiseNode(0.3, 1,
                                HeadingInterpolator.tangent.reverse())
                ))
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        // Bottom-row-disabled variant: same top-row pickup point, but swept in from the regular
        // station instead of BACK_POS. -12.300...° is THIS line's own reversed tangent --
        // recompute it if SHOOT_POS or the (22, 84) endpoint changes.
        toTopSweepFromRegular = f.pathBuilder()
                .addPath(new BezierLine(new Pose(SHOOT_POS_X, SHOOT_POS_Y), new Pose(22, 84)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, 0.3,
                                HeadingInterpolator.linear(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(-12.3000884507142132), 0.3)),
                        new HeadingInterpolator.PiecewiseNode(0.3, 1,
                                HeadingInterpolator.tangent.reverse())
                ))
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        // Returns to the TOP station at a constant SHOOT_HEADING_DEG the whole way. Shared by
        // both toTopSweep and toTopSweepFromRegular, since the pickup point and shoot station are
        // the same either way -- only how the robot gets to the pickup point differs.
        toShootFromTop = f.pathBuilder()
                .addPath(new BezierLine(new Pose(22, 84), new Pose(TOP_POS_X, TOP_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();
    }

    @Override
    public void run() {
        if (negabot == null) return;
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }

    private Pose initPose() {
        return new Pose(INIT_X, INIT_Y, Math.toRadians(INIT_HEADING_DEG));
    }

    /**
     * Turret window center for a shot where the robot arrives at {@code headingDeg} instead of
     * SHOOT_HEADING_DEG. A turret's required absolute angle is (roughly) chassis heading plus a
     * fixed bearing-to-goal offset, so it shifts 1:1 with heading -- shift DEFAULT_TURRET_DEG by
     * however far this shot's heading differs from the reference it was tuned at, rather than
     * hand-picking a whole new center for every differently-rotated shot.
     */
    private double turretCenterDeg(double headingDeg) {
        return DEFAULT_TURRET_DEG + (headingDeg - SHOOT_HEADING_DEG);
    }
}
