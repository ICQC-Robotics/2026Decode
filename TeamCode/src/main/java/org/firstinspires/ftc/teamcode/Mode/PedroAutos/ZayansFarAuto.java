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
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.ClampedPPTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.StallTimeoutCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitToShoot;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitUntilReadyToShoot;


@Autonomous(name = ".Ts will also work trust")
public class ZayansFarAuto extends CommandOpMode {

    Robot negabot;
    boolean isBlue   = true;
    boolean do3rdRow = true;
    private int zoneResult = 0;
    static final double SHOOT_POS_X        = 54.0;
    static final double SHOOT_POS_Y        = 14.0;
    static final double SHOOT_HEADING_DEG  = 0.0;
    static final double SHOOT_VELOCITY     = 3950;
    static final double SHOOT_HOOD         = 0.7;
    static final double SHOOT_TURRET_ANGLE = 22;
    static final double TURRET_WINDOW      = 20;

    static final double INIT_X           = 48;
    static final double INIT_Y           = 24;
    static final double INIT_HEADING_DEG = 0;

    static final double SWEEP_END_X = 8.5;
    static final double SWEEP_END_Y = 45.0;
    static final double SWEEP_CP_X  = 8.5;
    static final double SWEEP_CP_Y  = 19.5;
    static final double SWEEP_EXIT_HEADING_DEG = 271.1;

    static final long BURST_MS           = 600;
    // Max time to wait for the robot/turret to settle before shooting anyway (safety cap).
    static final long SHOOT_PAUSE_MS     = 500;
    static final double MAX_SETTLE_VELOCITY     = 2.0;  // in/sec, "stopped" threshold
    static final double TURRET_ALIGN_TOLERANCE  = 2.0;  // deg, "aligned" threshold
    static final long INTAKE_WAIT_MS     = 0;
    static final long STALL_TIMEOUT_MS   = 200;

    static final double COVER_OPEN  = 0.75;
    static final double COVER_CLOSE = 0.55;
    static final double INTAKE_ON   = -1.0;

    PathChain toHP, toShootFromHP, to3rdRow, toShootFrom3rdRow, toZone1, toShootFromZone1, toZone2, toShootFromZone2, toZone3, toShootFromZone3, toSweep, toShootFromSweep;

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, initPose());
        negabot.shooter.setMagazineCover(COVER_CLOSE);
        Robot.Alliance shown = Robot.Alliance.BLUE;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) isBlue   = true;
            if (gamepad1.b) isBlue   = false;
            if (gamepad1.y) do3rdRow = true;
            if (gamepad1.a) do3rdRow = false;
            negabot.vision.setAlliance(isBlue);

            Robot.Alliance sel = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
            if (sel != shown) {
                negabot.drive.follower.setPose(initPose());
                shown = sel;
            }
            negabot.drive.follower.update();
            Pose live = negabot.drive.follower.getPose();
            telemetry.addLine("=== FAR AUTO — SELECT OPTIONS, THEN PUSH TO START ===");
            telemetry.addData("Alliance", isBlue   ? ">>> BLUE <<<" : ">>>  RED <<<");
            telemetry.addData("3rd Row",  do3rdRow ? "ON  (A=disable)" : "OFF (Y=enable)");
            telemetry.addData("Live pose", String.format("x=%.1f y=%.1f h=%.0f",
                    live.getX(), live.getY(), Math.toDegrees(live.getHeading())));
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        Robot.ALLIANCE = isBlue ? Robot.Alliance.BLUE : Robot.Alliance.RED;
        negabot.turret.resetEncoder();

        Follower f = negabot.drive.follower;
        buildPaths(f);

        negabot.schedule(
                new InstantCommand(() -> negabot.shooter.setHood(SHOOT_HOOD)),
                new InstantCommand(() -> negabot.turret.setTargetDeg(td(SHOOT_TURRET_ANGLE))),
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new SequentialCommandGroup(
                        new WaitToShoot(negabot.intake, negabot.shooter, 3000),
                        burst(),

                        followGuarded(f, toHP, true),
                        new WaitCommand(INTAKE_WAIT_MS),
                        shotPrep(f, toShootFromHP, td(SHOOT_TURRET_ANGLE)),
                        new WaitUntilReadyToShoot(f, negabot.turret, MAX_SETTLE_VELOCITY, TURRET_ALIGN_TOLERANCE, SHOOT_PAUSE_MS),
                        burst(),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(f, to3rdRow, false),
                                        new WaitCommand(INTAKE_WAIT_MS),
                                        shotPrep(f, toShootFrom3rdRow, td(SHOOT_TURRET_ANGLE)),
                                        new WaitUntilReadyToShoot(f, negabot.turret, MAX_SETTLE_VELOCITY, TURRET_ALIGN_TOLERANCE, SHOOT_PAUSE_MS),
                                        burst()
                                ),
                                new InstantCommand(() -> {}),
                                () -> do3rdRow
                        ),

                        cycle(f),
                        cycle(f),
                        cycle(f),
                        cycle(f),
                        cycle(f),
                        cycle(f)
                )
        );
    }

    private Command cycle(Follower f) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> zoneResult = getZone()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                followGuarded(f, toZone1, false),
                                new WaitCommand(INTAKE_WAIT_MS),
                                shotPrep(f, toShootFromZone1, td(SHOOT_TURRET_ANGLE))
                        ),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        followGuarded(f, toZone2, false),
                                        new WaitCommand(INTAKE_WAIT_MS),
                                        shotPrep(f, toShootFromZone2, td(SHOOT_TURRET_ANGLE))
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                followGuarded(f, toZone3, false),
                                                new WaitCommand(INTAKE_WAIT_MS),
                                                shotPrep(f, toShootFromZone3, td(SHOOT_TURRET_ANGLE))
                                        ),
                                        new SequentialCommandGroup(
                                                followGuarded(f, toSweep, false),
                                                new WaitCommand(INTAKE_WAIT_MS),
                                                shotPrep(f, toShootFromSweep, td(SHOOT_TURRET_ANGLE))
                                        ),
                                        () -> zoneResult == 3
                                ),
                                () -> zoneResult == 2
                        ),
                        () -> zoneResult == 1
                ),
                new WaitUntilReadyToShoot(f, negabot.turret, MAX_SETTLE_VELOCITY, TURRET_ALIGN_TOLERANCE, SHOOT_PAUSE_MS),
                burst()
        );
    }

    private Command followGuarded(Follower f, PathChain path, boolean holdEnd) {
        return new ParallelRaceGroup(
                new FollowPathCommand(f, path, holdEnd),
                new StallTimeoutCommand(f, STALL_TIMEOUT_MS)
        );
    }

    Command shotPrep(Follower f, PathChain path, double centerDeg) {
        return new ParallelDeadlineGroup(
                new FollowPathCommand(f, path, true),
                new ClampedPPTracking(
                        negabot.turret,
                        negabot.drive,
                        Robot.ALLIANCE,
                        centerDeg - TURRET_WINDOW,
                        centerDeg + TURRET_WINDOW
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

    private int getZone() {
        int z = negabot.vision.getScannedZone(isBlue);
        double[] d = negabot.vision.getLastDensity(isBlue);
        telemetry.addData("density z1/z2/z3", "%.1f%% / %.1f%% / %.1f%%", d[1], d[2], d[3]);
        telemetry.addData("-> zone", z == 0 ? "SWEEP" : z);
        telemetry.update();
        return z;
    }

    void buildPaths(Follower f) {
        Pose pushedStart = f.getPose();


        toHP = f.pathBuilder()
                .addPath(new BezierLine(new Pose(pushedStart.getX(), pushedStart.getY()), sp(8.5, 9)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(0))
                .addPath(new BezierLine(sp(8.5, 9), sp(11.5, 9)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .addPath(new BezierLine(sp(11.5, 9), sp(8.5, 9)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        toShootFromHP = f.pathBuilder()
                .addPath(new BezierLine(sp(8.5, 9), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // 3rd row sweep (BezierCurve)
        to3rdRow = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(SHOOT_POS_X, SHOOT_POS_Y),
                        sp(43.56,   30),
                        sp(20,  38)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        toShootFrom3rdRow = f.pathBuilder()
                .addPath(new BezierLine(sp(20, 38), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(-12.4), hr(0))
                .build();

        // Zone 1 — HP corner (y ≈ 8). Single leg straight to the wall, so the follower actually
        // decelerates into it instead of blowing through at cruise speed mid-chain.
        toZone1 = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(8.5, 8)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();
        toShootFromZone1 = f.pathBuilder()
                .addPath(new BezierLine(sp(8.5, 8), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // Zone 2 (y = 20)
        toZone2 = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(8.5, 20)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();
        toShootFromZone2 = f.pathBuilder()
                .addPath(new BezierLine(sp(8.5, 20), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // Zone 3 (y = 32)
        toZone3 = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(8.5, 32)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();
        toShootFromZone3 = f.pathBuilder()
                .addPath(new BezierLine(sp(8.5, 32), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // Zone 0 — secret tunnel sweep (BezierCurve)
        toSweep = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(SHOOT_POS_X, SHOOT_POS_Y),
                        sp(SWEEP_CP_X,  SWEEP_CP_Y),
                        sp(SWEEP_END_X, SWEEP_END_Y)))
                .setTangentHeadingInterpolation().setReversed()
                .build();
        toShootFromSweep = f.pathBuilder()
                .addPath(new BezierLine(sp(SWEEP_END_X, SWEEP_END_Y), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(SWEEP_EXIT_HEADING_DEG), hr(0))
                .build();
    }

    @Override
    public void run() {
        if (negabot == null) return;
        super.run();
        Robot.LAST_POSE       = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }

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

    /** Mirrors a turret target angle for the current alliance (135=forward is the mirror axis). */
    private double td(double deg) {
        return isBlue ? deg : mtd(deg);
    }

    private static double mx(double x)    { return 144.0 - x; }
    private static double mhd(double deg) { return ((180.0 - deg) % 360.0 + 360.0) % 360.0; }
    private static double mtd(double deg) { return 270.0 - deg; }
}
