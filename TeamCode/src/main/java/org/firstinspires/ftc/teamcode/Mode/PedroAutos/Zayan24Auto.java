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


@Autonomous(name = "Ts gonna work trust")
public class Zayan24Auto extends CommandOpMode {

    Robot negabot;
    boolean isBlue = true;

    // ── Shooting-station tuning ─────────────────────────────────────────────
    // Near spot (40,90): preload, top row, bottom row  @ SHOOT_VELOCITY.
    // Far spot  (52,90): middle row + all gate cycles  @ SHOOT_VELOCITY_FAR.
    // Return paths already pass through (52,90), so far shots cost zero extra travel.
    static final double SHOOT_POS_X        = 61.845686512758206;
    static final double SHOOT_POS_Y        = 66.26002430133657;
    static final double SHOOT_HEADING_DEG  = 0;
    static final double SHOOT_VELOCITY    = 2950.0;
    static final double SHOOT_HOOD         = 0.5;
    static final double SHOOT_TURRET_OFFSET_DEG = 0.0;

    // ── Init / push-to-start ────────────────────────────────────────────────
    static final double INIT_X           = 48;
    static final double INIT_Y           = 120;
    static final double INIT_HEADING_DEG = 180;


    // ── Gate-intake-station tuning ──────────────────────────────────────────
    static final double GATE_POS_X       = 12.551032806804374;
    static final double GATE_POS_Y       = 55.70595382746051;
    static final double GATE_HEADING_DEG = 340.0;

    static final double GATE_PATH_ANGLE = 12.85;

    // ── Magazine cover positions ─────────────────────────────────────────────
    static final double COVER_OPEN  = .75;
    static final double COVER_CLOSE = .5;

    // ── Intake direction ─────────────────────────────────────────────────────
    static final double INTAKE_ON = -1.0;

    // ── Timing ──────────────────────────────────────────────────────────────
    static final long BURST_MS     = 550;
    static final long GATE_WAIT_MS = 800;

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
                new InstantCommand(() -> negabot.shooter.setHoodPosition(SHOOT_HOOD)),   // normal hood for the preload too

                new SequentialCommandGroup(

                        // ── 1: Preload — drive to the shoot spot, then shoot normally ──
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShoot0),   // drive to the spot while auto-aiming the turret
                        burst(),                 // stop, then fire like every other cycle

                        // ── 2: Middle row (y≈60) — returns to (52,90) @ 3050 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toMiddle, false),   // flow into the return, don't brake/hold
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShoot1),
                        burst(),

                        // ── 3: Gate first pass — from (52,90), return to (52,90) @ 3050 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGate),
                        burst(),

                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGate),
                        burst(),

                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGate),
                        burst(),

                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGate),
                        burst(),

                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromGate),
                        burst(),

                        // ── 4: Top row (y≈84) — from (52,90), returns to (40,90) @ 2950 RPM ─
                        new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                        new FollowPathCommand(f, toTopSweep, false),   // flow into the return, don't brake/hold
                        new InstantCommand(() -> negabot.intake.setSpeed(0)),
                        new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                        shotPrep(f, toShootFromTop),
                        burst()
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
        double tx = p.getX() + Turret.FORWARD_OFFSET_IN * Math.cos(h) - Turret.LATERAL_OFFSET_IN * Math.sin(h);
        double ty = p.getY() + Turret.FORWARD_OFFSET_IN * Math.sin(h) + Turret.LATERAL_OFFSET_IN * Math.cos(h);
        Pose goal = FieldConstants.goalAimPointForAlliance(isBlue);
        double bearingDeg    = Math.toDegrees(Math.atan2(goal.getY() - ty, goal.getX() - tx));
        double deflectionDeg = wrap180(bearingDeg - Math.toDegrees(h));
        negabot.turret.setTargetDeg(Turret.clampDeg(135.0 - deflectionDeg + SHOOT_TURRET_OFFSET_DEG));
    }

    void buildPaths(Follower f) {

        Pose pushedStart = f.getPose();

        // ── Start → near shoot spot ──────────────────────────────────────────
        toShoot0 = f.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pushedStart.getX(), pushedStart.getY()),
                        sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(SHOOT_HEADING_DEG))
                .build();

        // ── Middle row (y ≈ 60) ──────────────────────────────────────────────
        // 4-pt cubic: (40,90)→(52,90)→(52,60)→(36,60). Crosses y=84 at x≈47,
        // clearing the top-row artifact zone (x=18–36).
        toMiddle = f.pathBuilder()
                .addPath(new BezierLine(
                        sp(SHOOT_POS_X,     SHOOT_POS_Y),
                        sp(43.749999808076936, 62.06846346810267)))
                .setLinearHeadingInterpolation(hr(SHOOT_HEADING_DEG), hr(188.39))
                .addPath(new BezierLine(sp(43.749999808076936, 62.06846346810267), sp(21.570777450847295, 58.801756299208385)))
                .setTangentHeadingInterpolation()
                .build();

        // 3-pt return: (18,60)→ctrl(52,60)→(52,90). Ends at far shoot spot.
        toShoot1 = f.pathBuilder()
                .addPath(new BezierLine(
                        sp(21.570777450847295, 58.801756299208385),
                        sp(SHOOT_POS_X,     SHOOT_POS_Y)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        // ── Gate (y ≈ 61) ───────────────────────────────────────────────────
        // toGate: 4-pt cubic from (40,90). Crosses y=84 at x≈47. Used cycles 5 & 7.
        // Straight: top row is already collected by cycles 5 & 7, so go direct.
        toGate = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(21.49149453219927, 57.081214914945321992740947752126)))
                .setTangentHeadingInterpolation().setReversed()
                .addPath(new BezierLine(sp(21.49149453219927, 57.081214914945321992740947752126), sp(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_PATH_ANGLE), hr(GATE_HEADING_DEG))
                .build();


        // 3-pt return: (9.5,61)→ctrl(52,61)→(52,90). Ends at far shoot spot.
        toShootFromGate = f.pathBuilder()
                .addPath(new BezierLine( sp(GATE_POS_X, GATE_POS_Y), sp(21.49149453219927, 57.081214914945321992740947752126)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_PATH_ANGLE))
                .addPath(new BezierLine(sp(21.49149453219927, 57.081214914945321992740947752126), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setTangentHeadingInterpolation()
                .build();

        toTopSweep = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(36.277642770352365, 78.7446414034022)))
                .setLinearHeadingInterpolation(hr(GATE_PATH_ANGLE), hr(-18.8))
                .addPath(new BezierLine(sp(36.277642770352365, 78.7446414034022), sp(24.586269744835963, 82.69907639732685)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        toShootFromTop = f.pathBuilder()
                .addPath(new BezierLine(sp(24.586269744835963, 82.69907639732685), sp(36.33368814394519, 95.71666170041469)))
                .setLinearHeadingInterpolation(hr(-18.8), hr(47.9))
                .addPath(new BezierLine(sp(36.33368814394519, 95.71666170041469), sp(49.056652907007155, 109.12735428850704)))
                .setTangentHeadingInterpolation()
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

    private static double wrap180(double a) {
        a = ((a + 180.0) % 360.0 + 360.0) % 360.0;
        return a - 180.0;
    }

    private static double mx(double x)    { return 144.0 - x; }
    private static double mhd(double deg) { return ((180.0 - deg) % 360.0 + 360.0) % 360.0; }
}
