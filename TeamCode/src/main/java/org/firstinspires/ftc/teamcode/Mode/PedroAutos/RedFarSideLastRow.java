package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Vision.IntakeColorProcessing;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.TurretTracking;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

@Autonomous
public class RedFarSideLastRow extends CommandOpMode {
    Robot negabot;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
    PathChain ZoneAGo;
    PathChain ZoneABack;
    PathChain ZoneBGo;
    PathChain ZoneBBack;
    PathChain ZoneCGo;
    PathChain ZoneCBack;

    // Mirror rules:
    // Mirror about x = 72 => x' = 144 - x
    // Heading mirror      => heading' = pi - heading
    // Turret: 135 forward => turret' = 270 - turret   (NOTE: TurretTracking handles turret; no direct turret angles here)

    private static double mx(double x) { return 144.0 - x; }

    private static double mHeading(double rad) { return normRad(Math.PI - rad); }

    private static double normRad(double rad) {
        while (rad <= -Math.PI) rad += 2.0 * Math.PI;
        while (rad >  Math.PI)  rad -= 2.0 * Math.PI;
        return rad;
    }

    Pose startPose = new Pose(mx(57.05219206680585), 7.098121085594997, mHeading(Math.toRadians(0)));
    final double COVER_OPEN = 0.25;
    final double COVER_CLOSE = 1.0;

    Drive d;
    Follower f;
    Turret t;
    Intake intake;

    private IntakeColorProcessing.Zone latchedZone = null;

    // Track goal ALL THE TIME during the auto
    private TurretTracking ppTrack;

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, startPose);

        d = negabot.drive;
        f = d.follower;
        t = negabot.turret;
        intake = negabot.intake;

        t.resetEncoder();

        // IMPORTANT: this opmode is still "BLUE" alliance unless you intend otherwise.
        // Mirroring geometry does NOT automatically change alliance logic.
        Robot.ALLIANCE = Robot.Alliance.RED;

        // Create once and schedule to run continuously
        ppTrack = new TurretTracking(t, d, Robot.Alliance.RED);

        path(f);

        waitForStart();

        negabot.schedule(
                // keep tracking running
                ppTrack,

                // mirror shouldn't change this call; if decDeg() is a "trim" direction that assumes non-mirrored,
                // you may need to swap to incDeg() depending on what decDeg() means in your TurretTracking.
                new InstantCommand(() -> { ppTrack.incDeg(); }),

                // Everything else runs in parallel with tracking
                new InstantCommand(() -> { negabot.shooter.setVelocity(4050); }),
                new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_CLOSE); }),

                new SequentialCommandGroup(
                        new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_OPEN); }),

                        new FollowPathCommand(f, Path1, true),
                        new WaitCommand(1500),
                        shoot(),

                        new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path2, true),
                        new FollowPathCommand(f, Path3),

                        shotPrep(Path4, 0.1),
                        new WaitCommand(500),
                        shoot(),

                        new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path7, true),
                        new FollowPathCommand(f, Path8),
                        shotPrep(Path9, 0.1),

                        new WaitCommand(500),
                        shoot(),

                        latchZone(),
                        branchOnLatchedZone(),
                        latchZone(),
                        branchOnLatchedZone(),
                        latchZone(),
                        branchOnLatchedZone(),
                        latchZone(),
                        branchOnLatchedZone()

                )
        );
    }

    public Command shoot() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                new WaitCommand(500),
                new InstantCommand(() -> { negabot.intake.setSpeed(0); }),
                new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_CLOSE); })
        );
    }

    // turret tracking is now always-on, so this is just "drive + open cover" prep
    public Command shotPrep(PathChain path, double hoodPos) {
        return new ParallelCommandGroup(
                new FollowPathCommand(f, path, true),
                new WaitCommand(800),
                new InstantCommand(() -> { intake.setSpeed(0); }),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> { negabot.shooter.setMagazineCover(COVER_OPEN); })
                )
        );
    }

    private Command latchZone() {
        return new InstantCommand(() -> {
            latchedZone = negabot.vision.getZone();   // read RIGHT NOW
        });
    }

    private Command branchOnLatchedZone() {
        return new SelectCommand(
                java.util.Map.of(
                        IntakeColorProcessing.Zone.RIGHT, new SequentialCommandGroup(
                                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                                new FollowPathCommand(f, ZoneAGo, true),
                                shotPrep(ZoneABack, 0.1),
                                new WaitCommand(500),
                                shoot()
                        ),
                        IntakeColorProcessing.Zone.CENTER, new SequentialCommandGroup(
                                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                                new FollowPathCommand(f, ZoneBGo, true),
                                shotPrep(ZoneBBack, 0.1),
                                new WaitCommand(500),
                                shoot()
                        ),
                        IntakeColorProcessing.Zone.LEFT, new SequentialCommandGroup(
                                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                                new FollowPathCommand(f, ZoneCGo, true),
                                shotPrep(ZoneCBack, 0.1),
                                new WaitCommand(500),
                                shoot()
                        ),
                        IntakeColorProcessing.Zone.NONE, new SequentialCommandGroup(
                                new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                                new FollowPathCommand(f, ZoneCGo, true),
                                shotPrep(ZoneCBack, 0.1),
                                new WaitCommand(500),
                                shoot()
                        )
                ),
                () -> latchedZone
        );
    }

    public void path(Follower follower) {

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(56.935), 7.117),
                                new Pose(mx(55.795), 18.379)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(55.795), 18.379),
                                new Pose(mx(42.853), 35.735)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(42.853), 35.735),
                                new Pose(mx(11.375), 35.801)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.375), 35.801),
                                new Pose(mx(55.886), 18.599)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        ZoneAGo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(55.886), 18.599),
                                new Pose(mx(38.416), 5.943),
                                new Pose(mx(11.221), 7.880)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        ZoneABack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.221), 7.880),
                                new Pose(mx(55.640), 18.681)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        ZoneBGo = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(55.640), 18.681),
                                new Pose(mx(11.410), 17.726)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        ZoneBBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.410), 17.726),
                                new Pose(mx(55.590), 18.612)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        ZoneCGo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(55.590), 18.612),
                                new Pose(mx(50.720), 44.647),
                                new Pose(mx(11.118), 39.066)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        ZoneCBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.118), 39.066),
                                new Pose(mx(55.666), 18.637)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(mx(55.666), 18.637),
                                new Pose(mx(4.696), 11.957),
                                new Pose(mx(14.882), 21.907),
                                new Pose(mx(11.413), 38.924)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(270)))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.413), 38.924),
                                new Pose(mx(55.723), 18.625)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(270)), mHeading(Math.toRadians(0)))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(55.795), 18.379),
                                new Pose(mx(11.217), 8.403)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(57.052), 7.098),
                                new Pose(mx(11.217), 17.403)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.217), 17.403),
                                new Pose(mx(55.795), 18.379)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mx(11.217), 8.403),
                                new Pose(mx(55.795), 18.379)
                        )
                ).setLinearHeadingInterpolation(mHeading(Math.toRadians(0)), mHeading(Math.toRadians(0)))
                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}