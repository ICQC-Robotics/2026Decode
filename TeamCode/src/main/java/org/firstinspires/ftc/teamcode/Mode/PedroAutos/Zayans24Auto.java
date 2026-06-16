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
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;


@Autonomous(name = "Ts gonna work trust")
public class Zayans24Auto extends CommandOpMode {

    Robot negabot;
    boolean isBlue = true;

    // ── Shooting-station tuning ─────────────────────────────────────────────
    // Near spot (40,90): preload, top row, bottom row  @ SHOOT_VELOCITY.
    // Far spot  (52,90): middle row + all gate cycles  @ SHOOT_VELOCITY_FAR.
    // Return paths already pass through (52,90), so far shots cost zero extra travel.
    static final double SHOOT_POS_X        = 61.845686512758206;
    static final double SHOOT_POS_Y        = 66.26002430133657;
    static final double SHOOT_VELOCITY    = 3050;
    static final double SHOOT_HOOD         = 0.5;
    static final double SHOOT_TURRET_ANGLE = 19.6;

    // ── Init / push-to-start ────────────────────────────────────────────────
    static final double INIT_X           = 48;
    static final double INIT_Y           = 120;
    static final double INIT_HEADING_DEG = 180;


    // ── Gate-intake-station tuning ──────────────────────────────────────────
    static final double GATE_POS_X       = 10.5;
    static final double GATE_POS_Y       = 59;
    static final double GATE_HEADING_DEG = 333;

    static final double GATE_PATH_ANGLE = 12.85;

    // ── Magazine cover positions ─────────────────────────────────────────────
    static final double COVER_OPEN  = .8;
    static final double COVER_CLOSE = .5;

    // ── Intake direction ─────────────────────────────────────────────────────
    static final double INTAKE_ON = -1.0;

    // ── Timing ──────────────────────────────────────────────────────────────
    static final long BURST_MS     = 400;
    static final long GATE_WAIT_MS = 800;

    PathChain toShoot0;
    PathChain toMiddle,   toShoot1;
    PathChain toGate,     toShootFromGate;
    PathChain toTopSweep, toShootFromTop;

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


        // ── Command schedule ──────────────────────────────────────────────
        negabot.schedule(
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new InstantCommand(() -> negabot.turret.setTargetDeg(SHOOT_TURRET_ANGLE)),
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
                new InstantCommand(() -> negabot.shooter.setHood(SHOOT_HOOD)),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE)),
                new SequentialCommandGroup(
                        shotPrep(f, toShoot0, SHOOT_TURRET_ANGLE - 2.3),
                        burst(),

                        new FollowPathCommand(f, toMiddle, false),
                        shotPrep(f, toShoot1, SHOOT_TURRET_ANGLE - 2.3),
                        burst(),

                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, SHOOT_TURRET_ANGLE),
                        burst(),

                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, SHOOT_TURRET_ANGLE),
                        burst(),

                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, SHOOT_TURRET_ANGLE),
                        burst(),

                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, SHOOT_TURRET_ANGLE),
                        burst(),

                        new FollowPathCommand(f, toGate, true),
                        new WaitCommand(GATE_WAIT_MS),
                        shotPrep(f, toShootFromGate, SHOOT_TURRET_ANGLE),
                        burst(),

                        //TODO: These values are different, need to be tuned
                        new InstantCommand(() -> negabot.turret.setTargetDeg(38.8)),
                        new InstantCommand(() -> negabot.shooter.setVelocity(2600)),
                        new InstantCommand(() -> negabot.shooter.setHood(.35)),
                        new FollowPathCommand(f, toTopSweep, false),
                        shotPrep(f, toShootFromTop, 38.8),
                        burst()
                )
        );
    }

    Command burst() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new WaitCommand(BURST_MS),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    Command shotPrep(Follower f, PathChain path, double d) {
        return new ParallelDeadlineGroup(
                new FollowPathCommand(f, path, true),
                new RunCommand(() -> negabot.turret.setTargetDeg(d), negabot.turret)
        );
    }

    void buildPaths(Follower f) {

        Pose pushedStart = f.getPose();

        toShoot0 = f.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(pushedStart.getX(), pushedStart.getY()),
                        sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(8.39))
                .build();

        toMiddle = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(19.570777450847295, 58.801756299208385)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        toShoot1 = f.pathBuilder()
                .addPath(new BezierLine(
                        sp(21.570777450847295, 58.801756299208385),
                        sp(SHOOT_POS_X,     SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(8.39), hr(GATE_PATH_ANGLE), 1, 0.7)
                .build();

        toGate = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(GATE_POS_X, GATE_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_PATH_ANGLE), hr(GATE_HEADING_DEG))
                .build();

        toShootFromGate = f.pathBuilder()
                .addPath(new BezierLine(sp(GATE_POS_X, GATE_POS_Y), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(GATE_HEADING_DEG), hr(GATE_PATH_ANGLE), 0.3, 0.1)
                .build();

        toTopSweep = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(24.586269744835963, 82.69907639732685)))
                .setLinearHeadingInterpolation(hr(GATE_PATH_ANGLE), hr(-18.8), 0.3, 0)
                .build();

        toShootFromTop = f.pathBuilder()
                .addPath(new BezierLine(sp(24.586269744835963, 82.69907639732685), sp(49.056652907007155, 109.12735428850704)))
                .setLinearHeadingInterpolation(hr(-18.8), hr(47.9), 0.3, 0)
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