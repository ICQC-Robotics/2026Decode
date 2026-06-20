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

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.ClampedPPTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;


@Autonomous(name = "..Ts gonna work trust")
public class Zayans24Auto extends CommandOpMode {

    Robot negabot;
    boolean isBlue = true;
    boolean do3rdRow = true;

    // ── Shooting-station tuning ─────────────────────────────────────────────
    // Cycle order: preload, middle row, gate, gate, 3rd row (toggleable),
    // gate, top row, gate.
    static final double SHOOT_POS_X        = 54.0672708+2;
    static final double SHOOT_POS_Y        = 71.57207864719906+5;
    static final double SHOOT_HEADING_DEG  = 46;
    static final double SHOOT_VELOCITY     = 3050;
    static final double SHOOT_HOOD         = 0.5;
    static final double DEFAULT_TURRET_DEG = 52;
    static final double GATE_TURRET_DEG    = 19;
    static final double TURRET_WINDOW      = 20;

    // ── RPM gate (preload only) ───────────────────────────────────────────
    static final double RPM_TOLERANCE       = 100;
    static final long   RPM_WAIT_TIMEOUT_MS = 1500;

    // ── Init / push-to-start ────────────────────────────────────────────────
    static final double INIT_X           = 48;
    static final double INIT_Y           = 120;
    static final double INIT_HEADING_DEG = 180;


    // ── Gate-intake-station tuning ──────────────────────────────────────────
    static final double GATE_POS_X       = 9.5;
    static final double GATE_POS_Y       = 59;
    static final double GATE_HEADING_DEG = 340;

    // ── Spike-row intake reach ──────────────────────────────────────────────
    static final double spikeXintake = 10;

    // ── Magazine cover positions ─────────────────────────────────────────────
    static final double COVER_OPEN  = .75;
    static final double COVER_CLOSE = .55;

    // ── Intake direction ─────────────────────────────────────────────────────
    static final double INTAKE_ON = -1.0;

    // ── Timing ──────────────────────────────────────────────────────────────
    static final long BURST_MS     = 425;
    static final long GATE_WAIT_MS = 950;
    static final long FAST_GATE_WAIT_MS = 450;

    // Low brakingStart -> the chain keeps cruising at full speed and only decelerates right at the
    // very end, instead of slowing down early -- used on paths that drive into an intake/row/gate.
    static final double INTAKE_BRAKING_START = 0.05;

    PathChain toShoot0;
    PathChain toMiddle,   toShoot1;
    PathChain toGate,     toShootFromGate;
    PathChain toTopSweep, toShootFromTop;
    PathChain toBottom,   toShootFromBottom;

    @Override
    public void initialize() {
        // ── Alliance selection ────────────────────────────────────────────
        negabot = new Robot(hardwareMap, telemetry, initPose());
        negabot.shooter.setMagazineCover(COVER_CLOSE);
        Robot.Alliance shown = Robot.Alliance.BLUE;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue = true;
            if (gamepad1.b) isBlue = false;
            if (gamepad1.y) do3rdRow = true;
            if (gamepad1.a) do3rdRow = false;
            Robot.Alliance sel = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
            if (sel != shown) {
                negabot.drive.follower.setPose(initPose());
                shown = sel;
            }
            negabot.drive.follower.update();
            Pose live = negabot.drive.follower.getPose();
            telemetry.addLine("=== SELECT ALLIANCE (X/B), THEN PUSH TO START ===");
            telemetry.addData("Selected", isBlue ? ">>> BLUE <<<" : ">>>  RED <<<");
            telemetry.addData("3rd Row", do3rdRow ? "ON  (A=disable)" : "OFF (Y=enable)");
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


        // ── Command schedule ──────────────────────────────────────────────
        negabot.schedule(
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new InstantCommand(() -> negabot.turret.setTargetDeg(DEFAULT_TURRET_DEG)),
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
                new InstantCommand(() -> negabot.shooter.setHood(SHOOT_HOOD)),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE)),
                new SequentialCommandGroup(
                        // ── Preload (cycle 1) ──
                        shotPrep(f, toShoot0, DEFAULT_TURRET_DEG - 1, TURRET_WINDOW),
                        waitForRpm(),
                        burst(),

                        // ── Middle row (cycle 2) ──
                        new FollowPathCommand(f, toMiddle, false),
                        shotPrep(f, toShoot1, DEFAULT_TURRET_DEG, TURRET_WINDOW),
                        burst(),

                        // ── Gate 1 (cycle 3) ──
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(FAST_GATE_WAIT_MS - 150),
                        shotPrep(f, toShootFromGate, GATE_TURRET_DEG, TURRET_WINDOW),
                        burst(),

                        // ── Gate 2 (cycle 4) ──
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, GATE_TURRET_DEG, TURRET_WINDOW),
                        burst(),

                        // ── Top row (cycle 5) — returns to the regular shoot spot ──
                        new FollowPathCommand(f, toTopSweep, false),
                        shotPrep(f, toShootFromTop, DEFAULT_TURRET_DEG, TURRET_WINDOW),
                        burst(),

                        // ── Gate 3 (cycle 6) ──
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(FAST_GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, GATE_TURRET_DEG, TURRET_WINDOW),
                        burst(),

                        // ── Gate 3 (cycle 6) ──
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, GATE_TURRET_DEG, TURRET_WINDOW),
                        burst(),

                        // ── 3rd row (cycle 7, toggleable, defaults ON) ──
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(f, toBottom, false),
                                        shotPrep(f, toShootFromBottom, DEFAULT_TURRET_DEG, TURRET_WINDOW),
                                        burst()
                                ),
                                new InstantCommand(() -> {}),
                                () -> do3rdRow
                        ),

                        // ── Gate 4 (cycle 8) ──
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(FAST_GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, GATE_TURRET_DEG, TURRET_WINDOW),
                        burst()
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

    void buildPaths(Follower f) {

        Pose pushedStart = f.getPose();

        toShoot0 = f.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pushedStart.getX(), pushedStart.getY()),
                        sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(SHOOT_HEADING_DEG))
                .build();

        toGate = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(GATE_HEADING_DEG))
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        toShootFromGate = f.pathBuilder()
                .addPath(new BezierLine(sp(GATE_POS_X, GATE_POS_Y), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(SHOOT_HEADING_DEG), 0.3, 0.1)
                .build();

        // ── Middle row (y ≈ 58.8) — curved sweep, mirrors CloseAuto18Leave's row-curve pattern ──
        toMiddle = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(SHOOT_POS_X, SHOOT_POS_Y),
                        sp(45, 58.801756299208385),
                        sp(20.570777450847295, 58.801756299208385)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        toShoot1 = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(20.570777450847295, 58.801756299208385),
                        sp(45, 58.801756299208385),
                        sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(SHOOT_HEADING_DEG))
                .build();

        // Single straight line to (18.7, 82.3). True tangent/reversed would snap instantly from
        // the incoming 46° (the "wild swing"), so the first 30% (in time, not distance) ramps
        // heading linearly to the path's back-leading angle; the remaining 70% follows that same
        // line tangentially/reversed.
        toTopSweep = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(18.7, 82.3)))
                .setHeadingInterpolation(HeadingInterpolator.piecewise(
                        new HeadingInterpolator.PiecewiseNode(0, 0.3,
                                HeadingInterpolator.linear(hr(SHOOT_HEADING_DEG), hr(-8.7148699854665779), 0.3)),
                        new HeadingInterpolator.PiecewiseNode(0.3, 1,
                                HeadingInterpolator.tangent.reverse())
                ))
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        // Returns to the regular shoot spot at a constant SHOOT_HEADING_DEG the whole way (no ramp
        // from the sweep's exit heading) — same flat-heading pattern as toShootFromBottom.
        toShootFromTop = f.pathBuilder()
                .addPath(new BezierLine(sp(18.7, 82.3), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(SHOOT_HEADING_DEG))
                .build();

        // ── 3rd row / bottom row (y ≈ 36) — toggleable, defaults ON ──────────
        toBottom = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(SHOOT_POS_X, SHOOT_POS_Y),
                        sp(45, 36.0),
                        sp(spikeXintake, 36.0)))
                .setTangentHeadingInterpolation().setReversed()
                .setGlobalDeceleration(INTAKE_BRAKING_START)
                .build();

        toShootFromBottom = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(spikeXintake, 36.0),
                        sp(45, 36.0),
                        sp(SHOOT_POS_X, SHOOT_POS_Y)))
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


    private static double mx(double x)    { return 144.0 - x; }
    private static double mhd(double deg) { return ((180.0 - deg) % 360.0 + 360.0) % 360.0; }
}