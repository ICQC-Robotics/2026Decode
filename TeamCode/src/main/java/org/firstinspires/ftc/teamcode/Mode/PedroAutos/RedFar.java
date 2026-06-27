package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.ClampedPPTracking;
import org.firstinspires.ftc.teamcode.Robot.commands.DriveToXYCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.StallTimeoutCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.VisionGuidedZoneApproach;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitToShoot;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitUntilReadyToShoot;

/**
 * Red-alliance mirror of BlueFar (itself split from ZayansFarAuto) -- fixed to RED, no alliance
 * prompt, no mirroring helpers. Every position constant below is BlueFar's value run through
 * x -> 144-x; every heading constant through deg -> 180-deg; SHOOT_TURRET_ANGLE through
 * deg -> 270-deg (135=forward is that mirror axis). One subtlety: the "+1.5" tweak on top of
 * SHOOT_TURRET_ANGLE at the initial pre-aim flips sign under that 270-deg transform
 * (mtd(x+1.5) = mtd(x)-1.5), so it's "-1.5" here where BlueFar has "+1.5" -- intentional, not a
 * typo. The per-shot turretOffsetDeg (-1) is a physical/mechanical correction, NOT mirrored, so
 * it's identical in both files. Behavior is otherwise identical to BlueFar; see that file for the
 * non-mirrored version.
 */
@Autonomous(name = "RedFar")
public class RedFar extends CommandOpMode {

    Robot negabot;
    boolean do3rdRow = true;
    private int zoneResult = 0;

    // ── intake anti-jam (far-auto only) ──────────────────────────────────────
    // Two artifacts entering at once can jam the intake, spiking motor current.
    // When the current exceeds the threshold, run the intake in reverse for a
    // set time to clear it, then resume what it was doing.
    public static boolean INTAKE_ANTI_JAM_ENABLED      = true;
    public static double  INTAKE_JAM_CURRENT_THRESHOLD = 7.0; // amps
    public static double  INTAKE_REVERSE_TIME          = 0.5; // seconds
    private DcMotorEx intakeMotor;
    private boolean intakeReversing = false;
    private double  intakeSavedPower = 0;
    private final ElapsedTime intakeReverseTimer = new ElapsedTime();

    static final double SHOOT_POS_X        = 90.0;
    static final double SHOOT_POS_Y        = 14.0;
    static final double SHOOT_HEADING_DEG  = 180.0;
    static final double SHOOT_VELOCITY     = 4000;
    static final double SHOOT_HOOD         = 0.7;
    // Mirror of BlueFar's SHOOT_TURRET_ANGLE (20.25) via deg -> 270-deg.
    static final double SHOOT_TURRET_ANGLE = 249.75;
    static final double TURRET_WINDOW      = 20;

    static final double INIT_X           = 96;
    static final double INIT_Y           = 24;
    static final double INIT_HEADING_DEG = 180;

    static final double SWEEP_END_X = 135.5;
    static final double SWEEP_END_Y = 45.0;
    static final double SWEEP_CP_X  = 135.5;
    static final double SWEEP_CP_Y  = 19.5;
    static final double SWEEP_EXIT_HEADING_DEG = 268.9;

    // to3rdRow's actual exit heading (tangent of its curve at the endpoint, reversed). Mirror of
    // BlueFar's THIRD_ROW_EXIT_HEADING_DEG (0.0) via deg -> 180-deg. Recompute this any time
    // to3rdRow's control point or endpoint changes -- it does NOT update itself.
    static final double THIRD_ROW_EXIT_HEADING_DEG = 180.0;

    // Fraction of a curve's t-range over which heading eases from the guaranteed incoming
    // heading into true reversed-tangent-tracking, instead of snapping straight to the tangent
    // heading at t=0 (which can be far from the heading the robot is actually arriving with).
    static final double CURVE_HEADING_EASE_T = 0.15;

    static final long BURST_MS           = 600;
    // Max time to wait for the robot/turret to settle before shooting anyway (safety cap).
    static final long SHOOT_PAUSE_MS     = 500;
    static final double MAX_SETTLE_VELOCITY     = 0.25;  // in/sec, "stopped" threshold
    static final double TURRET_ALIGN_TOLERANCE  = 1;  // deg, "aligned" threshold
    static final long INTAKE_WAIT_MS     = 0;
    static final long STALL_TIMEOUT_MS   = 100;
    // toHP's forward/backward bump needs to decelerate to ~0 and reverse direction mid-path,
    // which a 200ms stall window mistakes for actually being stuck -- give it more slack.
    static final long HP_BUMP_STALL_TIMEOUT_MS = 600;
    // The sweep curve is long and slow relative to other paths, so its stall window needs more
    // slack than the default to not mistake a brief slowdown for actually being stuck.
    static final long SWEEP_STALL_TIMEOUT_MS = 400;

    static final double COVER_OPEN  = 0.75;
    static final double COVER_CLOSE = 0.486;
    static final double INTAKE_ON   = -1.0;

    // Mid-path zone correction: while driving toward a picked zone, keep re-checking vision. If
    // the densest zone changes to something else, abandon the current path and snap to zone 1.
    static final long   ZONE_RESAMPLE_MS    = 200;  // how often to re-check vision while en route
    static final double ZONE_COMMIT_DIST_IN = 12.0; // stop retargeting once this close to the target
    // Brief dwell once arrived at a zone/sweep endpoint, before driving back to shoot.
    static final long   ZONE_ARRIVAL_WAIT_MS = 100;
    // While still approaching a zone (not yet on the way back) and within this far of the wall,
    // cap drive power for better control.
    static final double SLOW_ZONE_X_IN  = 24.0;
    static final double SLOW_ZONE_POWER = 1;

    // Safety net: no matter what's happening or where the robot is, abandon it at this point in
    // the match and park at the init spot -- heading doesn't matter, only getting there does.
    static final long   PARK_AT_MS = 29500;
    static final double PARK_X = 96;
    static final double PARK_Y = 24;

    PathChain toHP, toShootFromHP, to3rdRow, toShootFrom3rdRow, toSweep, toShootFromSweep;
    Pose[] zoneEndpoints; // index 1..5, the 5 swappable pickup lanes (1/3/5 original, 2/4 new in-between)

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, initPose(), true);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        negabot.shooter.setMagazineCover(COVER_CLOSE);
        negabot.vision.setAlliance(false);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.y) do3rdRow = true;
            if (gamepad1.a) do3rdRow = false;

            negabot.drive.follower.update();
            Pose live = negabot.drive.follower.getPose();
            telemetry.addLine("=== RED FAR — PUSH TO START ===");
            telemetry.addData("3rd Row",  do3rdRow ? "ON  (A=disable)" : "OFF (Y=enable)");
            telemetry.addData("Live pose", String.format("x=%.1f y=%.1f h=%.0f",
                    live.getX(), live.getY(), Math.toDegrees(live.getHeading())));
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) return;

        Robot.ALLIANCE = Robot.Alliance.RED;
        negabot.turret.resetEncoder();

        Follower f = negabot.drive.follower;
        buildPaths(f);

        Command mainSequence = new SequentialCommandGroup(
                new WaitToShoot(negabot.intake, negabot.shooter, 3000),
                burst(),

                followGuarded(f, toHP, false, HP_BUMP_STALL_TIMEOUT_MS),
                new WaitCommand(INTAKE_WAIT_MS),
                shotPrep(f, toShootFromHP, SHOOT_TURRET_ANGLE, -2),
                new WaitUntilReadyToShoot(f, negabot.turret, MAX_SETTLE_VELOCITY, TURRET_ALIGN_TOLERANCE, SHOOT_PAUSE_MS),
                burst(),

                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new FollowPathCommand(f, to3rdRow, false),
                                new WaitCommand(INTAKE_WAIT_MS),
                                shotPrep(f, toShootFrom3rdRow, SHOOT_TURRET_ANGLE, -2),
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
        );

        negabot.schedule(
                new InstantCommand(() -> negabot.shooter.setHood(SHOOT_HOOD)),
                new InstantCommand(() -> negabot.turret.setTargetDeg(SHOOT_TURRET_ANGLE - 2)),
                new InstantCommand(() -> negabot.shooter.setVelocity(SHOOT_VELOCITY)),
                new InstantCommand(() -> negabot.intake.setSpeed(INTAKE_ON)),
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                mainSequence,
                                new WaitCommand(PARK_AT_MS)
                        ),
                        new DriveToXYCommand(f, PARK_X, PARK_Y, true)
                )
        );
    }

    private Command cycle(Follower f) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> zoneResult = getZone()),
                new ConditionalCommand(
                        shotPrep(f, new VisionGuidedZoneApproach(
                                f, negabot.vision, false, zoneEndpoints, Math.toRadians(SHOOT_HEADING_DEG),
                                () -> zoneResult, ZONE_RESAMPLE_MS, ZONE_COMMIT_DIST_IN,
                                new Pose(SHOOT_POS_X, SHOOT_POS_Y, Math.toRadians(SHOOT_HEADING_DEG)), SLOW_ZONE_X_IN, SLOW_ZONE_POWER),
                                SHOOT_TURRET_ANGLE, -2),
                        new SequentialCommandGroup(
                                followGuarded(f, toSweep, false, SWEEP_STALL_TIMEOUT_MS),
                                new WaitCommand(ZONE_ARRIVAL_WAIT_MS),
                                shotPrep(f, toShootFromSweep, SHOOT_TURRET_ANGLE, -2)
                        ),
                        () -> zoneResult != 0
                ),
                new WaitUntilReadyToShoot(f, negabot.turret, MAX_SETTLE_VELOCITY, TURRET_ALIGN_TOLERANCE, SHOOT_PAUSE_MS),
                burst()
        );
    }

    private Command followGuarded(Follower f, PathChain path, boolean holdEnd) {
        return followGuarded(f, path, holdEnd, STALL_TIMEOUT_MS);
    }

    private Command followGuarded(Follower f, PathChain path, boolean holdEnd, long stallTimeoutMs) {
        return new ParallelRaceGroup(
                new FollowPathCommand(f, path, holdEnd),
                new StallTimeoutCommand(f, stallTimeoutMs)
        );
    }

    Command shotPrep(Follower f, PathChain path, double centerDeg) {
        return shotPrep(f, path, centerDeg, 0);
    }

    Command shotPrep(Follower f, PathChain path, double centerDeg, double turretOffsetDeg) {
        return shotPrep(f, new FollowPathCommand(f, path, true), centerDeg, turretOffsetDeg);
    }

    Command shotPrep(Follower f, Command driveCommand, double centerDeg) {
        return shotPrep(f, driveCommand, centerDeg, 0);
    }

    // turretOffsetDeg is a plain (non-alliance-mirrored) correction added straight into
    // ClampedPPTracking's angle formula before clamping -- centerDeg only sets the clamp window,
    // so nudging it does nothing unless the live computed angle is already at that window's edge.
    Command shotPrep(Follower f, Command driveCommand, double centerDeg, double turretOffsetDeg) {
        ClampedPPTracking tracking = new ClampedPPTracking(
                negabot.turret,
                negabot.drive,
                Robot.ALLIANCE,
                centerDeg - TURRET_WINDOW,
                centerDeg + TURRET_WINDOW
        );
        tracking.offset = turretOffsetDeg;
        return new ParallelDeadlineGroup(driveCommand, tracking);
    }

    Command burst() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_OPEN)),
                new WaitCommand(BURST_MS),
                new InstantCommand(() -> negabot.shooter.setMagazineCover(COVER_CLOSE))
        );
    }

    private int getZone() {
        int z = negabot.vision.getScannedZone(false);
        double[] d = negabot.vision.getLastDensity(false);
        telemetry.addData("density z1/z2/z3/z4/z5", "%.1f%% / %.1f%% / %.1f%% / %.1f%% / %.1f%%",
                d[1], d[2], d[3], d[4], d[5]);
        telemetry.addData("-> zone", z == 0 ? "SWEEP" : z);
        telemetry.update();
        return z;
    }

    void buildPaths(Follower f) {
        Pose pushedStart = f.getPose();


        toHP = f.pathBuilder()
                .addPath(new BezierLine(new Pose(pushedStart.getX(), pushedStart.getY()), new Pose(135.5, 8)))
                .setLinearHeadingInterpolation(pushedStart.getHeading(), Math.toRadians(SHOOT_HEADING_DEG))
                .addPath(new BezierLine(new Pose(135.5, 8), new Pose(132.5, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .addPath(new BezierLine(new Pose(132.5, 8), new Pose(135.5, 8)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();

        toShootFromHP = f.pathBuilder()
                .addPath(new BezierLine(new Pose(135.5, 8), new Pose(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SHOOT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();

        // 3rd row sweep (BezierCurve). Tangent-tracks the curve (reversed) like before, but eases
        // into that tracking over the first CURVE_HEADING_EASE_T of the path instead of snapping
        // straight to the tangent heading at t=0. Lands on THIRD_ROW_EXIT_HEADING_DEG at t=1.
        BezierCurve thirdRowCurve = new BezierCurve(
                new Pose(SHOOT_POS_X, SHOOT_POS_Y),
                new Pose(100.44, 36),
                new Pose(124,    36));
        to3rdRow = f.pathBuilder()
                .addPath(thirdRowCurve)
                .setHeadingInterpolation(easedReversedTangent(thirdRowCurve, Math.toRadians(SHOOT_HEADING_DEG), CURVE_HEADING_EASE_T))
                .build();

        toShootFrom3rdRow = f.pathBuilder()
                .addPath(new BezierLine(new Pose(124, 36), new Pose(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(THIRD_ROW_EXIT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();

        // Zones 1-5 (HP corner, mid, far, plus 2 new lanes between them -- y = 8/14/20/26/32).
        // VisionGuidedZoneApproach drives to whichever of these looks best and can re-target
        // (to zone 1) mid-flight, so only the endpoints are needed here; the approach/return
        // paths are built live at runtime.
        zoneEndpoints = new Pose[6];
        zoneEndpoints[1] = new Pose(135.5, 8);
        zoneEndpoints[2] = new Pose(135.5, 14);
        zoneEndpoints[3] = new Pose(135.5, 20);
        zoneEndpoints[4] = new Pose(135.5, 26);
        zoneEndpoints[5] = new Pose(135.5, 32);

        // Zone 0 — secret tunnel sweep (BezierCurve). Same eased-tangent fix as to3rdRow.
        BezierCurve sweepCurve = new BezierCurve(
                new Pose(SHOOT_POS_X, SHOOT_POS_Y),
                new Pose(SWEEP_CP_X,  SWEEP_CP_Y),
                new Pose(SWEEP_END_X, SWEEP_END_Y));
        toSweep = f.pathBuilder()
                .addPath(sweepCurve)
                .setHeadingInterpolation(easedReversedTangent(sweepCurve, Math.toRadians(SHOOT_HEADING_DEG), CURVE_HEADING_EASE_T))
                .build();
        toShootFromSweep = f.pathBuilder()
                .addPath(new BezierLine(new Pose(SWEEP_END_X, SWEEP_END_Y), new Pose(SHOOT_POS_X, SHOOT_POS_Y)))
                .setLinearHeadingInterpolation(Math.toRadians(SWEEP_EXIT_HEADING_DEG), Math.toRadians(SHOOT_HEADING_DEG))
                .build();
    }

    /**
     * Tangent-tracks {@code curve}, reversed (robot drives backward along it), except over the
     * first {@code easeT} of the path: there, heading linearly eases from {@code entryHeadingRad}
     * to wherever the reversed tangent actually is at {@code easeT}, so the curve doesn't demand
     * an instant snap-rotation at t=0 to whatever its tangent happens to be there. From
     * {@code easeT} to 1 this is identical to {@code setTangentHeadingInterpolation().setReversed()}.
     */
    private HeadingInterpolator easedReversedTangent(BezierCurve curve, double entryHeadingRad, double easeT) {
        double reversedAtEaseT = curve.getDerivative(easeT).getTheta() + Math.PI;
        return HeadingInterpolator.piecewise(
                HeadingInterpolator.PiecewiseNode.linear(0, easeT, entryHeadingRad, reversedAtEaseT),
                new HeadingInterpolator.PiecewiseNode(easeT, 1, HeadingInterpolator.tangent.reverse())
        );
    }

    @Override
    public void run() {
        if (negabot == null) return;
        super.run();
        intakeAntiJam();
        Robot.LAST_POSE       = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }

    /** Reverse the intake briefly when its current spikes from a jam. */
    private void intakeAntiJam() {
        if (intakeMotor == null || !INTAKE_ANTI_JAM_ENABLED) return;
        if (intakeReversing) {
            if (intakeReverseTimer.seconds() >= INTAKE_REVERSE_TIME) {
                intakeReversing = false;
                intakeMotor.setPower(intakeSavedPower);   // resume what we interrupted
            } else {
                intakeMotor.setPower(-intakeSavedPower);  // hold reverse vs. command writes
            }
        } else {
            double power = intakeMotor.getPower();
            if (power != 0 && intakeMotor.getCurrent(CurrentUnit.AMPS) > INTAKE_JAM_CURRENT_THRESHOLD) {
                intakeSavedPower = power;
                intakeReversing = true;
                intakeReverseTimer.reset();
                intakeMotor.setPower(-power);
            }
        }
    }

    private Pose initPose() {
        return new Pose(INIT_X, INIT_Y, Math.toRadians(INIT_HEADING_DEG));
    }
}
