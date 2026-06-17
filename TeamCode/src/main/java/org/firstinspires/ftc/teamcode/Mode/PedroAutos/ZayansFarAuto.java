package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;


@Autonomous(name = "Ts will also work trust")
public class ZayansFarAuto extends CommandOpMode {

    Robot negabot;
    boolean isBlue   = true;
    boolean do3rdRow = false;
    private int zoneResult = 0;
    static final double SHOOT_POS_X        = 45.0;
    static final double SHOOT_POS_Y        = 9.0;
    static final double SHOOT_HEADING_DEG  = 0.0;
    static final double SHOOT_VELOCITY     = 3950;
    static final double SHOOT_HOOD         = 0.7;
    static final double SHOOT_TURRET_ANGLE = 26.6;

    static final double INIT_X           = 48;
    static final double INIT_Y           = 24;
    static final double INIT_HEADING_DEG = 0;

    static final double SWEEP_END_X = 9.0;
    static final double SWEEP_END_Y = 45.0;
    static final double SWEEP_CP_X  = 9.5;
    static final double SWEEP_CP_Y  = 19.5;
    static final double SWEEP_EXIT_HEADING_DEG = 271.1;

    static final long BURST_MS        = 600;
    static final long INTAKE_WAIT_MS  = 300;

    static final double COVER_OPEN  = 0.75;
    static final double COVER_CLOSE = 0.5;
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
                new InstantCommand(() -> negabot.turret.setTargetDeg(SHOOT_TURRET_ANGLE)),
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        burst(),

                        new FollowPathCommand(f, toHP, true),
                        new WaitCommand(INTAKE_WAIT_MS),
                        shotPrep(f, toShootFromHP, SHOOT_TURRET_ANGLE),
                        burst(),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(f, to3rdRow, false),
                                        new WaitCommand(INTAKE_WAIT_MS),
                                        shotPrep(f, toShootFrom3rdRow, SHOOT_TURRET_ANGLE),
                                        burst()
                                ),
                                new InstantCommand(() -> {}),
                                () -> do3rdRow
                        ),

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
                                new FollowPathCommand(f, toZone1, false),
                                new WaitCommand(INTAKE_WAIT_MS),
                                new FollowPathCommand(f, toShootFromZone1, true)
                        ),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(f, toZone2, false),
                                        new WaitCommand(INTAKE_WAIT_MS),
                                        new FollowPathCommand(f, toShootFromZone2, true)
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(f, toZone3, false),
                                                new WaitCommand(INTAKE_WAIT_MS),
                                                new FollowPathCommand(f, toShootFromZone3, true)
                                        ),
                                        new SequentialCommandGroup(
                                                new FollowPathCommand(f, toSweep, false),
                                                new WaitCommand(INTAKE_WAIT_MS),
                                                new FollowPathCommand(f, toShootFromSweep, true)
                                        ),
                                        () -> zoneResult == 3
                                ),
                                () -> zoneResult == 2
                        ),
                        () -> zoneResult == 1
                ),
                burst()
        );
    }

    Command shotPrep(Follower f, PathChain path, double d) {
        return new ParallelDeadlineGroup(
                new FollowPathCommand(f, path, true),
                new RunCommand(() -> negabot.turret.setTargetDeg(d), negabot.turret)
        );
    }

    Command burst() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                new WaitCommand(BURST_MS),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    //TODO: Implement
    private int getZone() {
        int z = negabot.vision.scanArtifactZone(isBlue);
        double[] d = negabot.vision.getLastDensity();
        telemetry.addData("density z1/z2/z3", "%.1f / %.1f / %.1f", d[1], d[2], d[3]);
        telemetry.addData("-> zone", z == 0 ? "SWEEP" : z);
        telemetry.update();
        return z;
    }
    void buildPaths(Follower f) {
        Pose pushedStart = f.getPose();


        toHP = f.pathBuilder()
                .addPath(new BezierLine(new Pose(pushedStart.getX(), pushedStart.getY()), sp(8, 9)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), hr(0))
                .build();

        toShootFromHP = f.pathBuilder()
                .addPath(new BezierLine(sp(8, 9), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // 3rd row sweep (BezierCurve)
        to3rdRow = f.pathBuilder()
                .addPath(new BezierCurve(
                        sp(SHOOT_POS_X, SHOOT_POS_Y),
                        sp(43.56,   30),
                        sp(24.7,  34.1)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

        toShootFrom3rdRow = f.pathBuilder()
                .addPath(new BezierLine(sp(24.7, 34.1), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(-12.4), hr(0))
                .build();

        // Zone 1 — HP corner (y ≈ 8)
        toZone1 = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(8, 8)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();
        toShootFromZone1 = f.pathBuilder()
                .addPath(new BezierLine(sp(8, 8), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // Zone 2 (y = 20)
        toZone2 = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(8, 20)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();
        toShootFromZone2 = f.pathBuilder()
                .addPath(new BezierLine(sp(8, 20), sp(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();

        // Zone 3 (y = 32)
        toZone3 = f.pathBuilder()
                .addPath(new BezierLine(sp(SHOOT_POS_X, SHOOT_POS_Y), sp(8, 32)))
                .setLinearHeadingInterpolation(hr(0), hr(0))
                .build();
        toShootFromZone3 = f.pathBuilder()
                .addPath(new BezierLine(sp(8, 32), sp(SHOOT_POS_X, SHOOT_POS_Y)))
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

    private static double mx(double x)    { return 144.0 - x; }
    private static double mhd(double deg) { return ((180.0 - deg) % 360.0 + 360.0) % 360.0; }
}
