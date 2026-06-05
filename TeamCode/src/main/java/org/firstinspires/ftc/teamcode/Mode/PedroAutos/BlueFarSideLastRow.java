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
import org.firstinspires.ftc.teamcode.Robot.commands.ArtifactSeekCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.PPTracking;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

@Autonomous
public class BlueFarSideLastRow extends CommandOpMode {
    Robot negabot;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
    PathChain ZoneAGo;
    PathChain ZoneABack;
    PathChain ZoneBGo;
    PathChain ZoneBBack;
    PathChain ZoneCGo;
    PathChain ZoneCBack;

    Pose startPose = new Pose(57.05219206680585, 7.098121085594997, Math.toRadians(0));
    final double COVER_OPEN = 0.25;
    final double COVER_CLOSE = 1.0;

    Drive d;
    Follower f;
    Turret t;
    Intake intake;

    private IntakeColorProcessing.Zone latchedZone = null;

    // Track goal ALL THE TIME during the auto
    private PPTracking ppTrack;

    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, startPose);

        d = negabot.drive;
        f = d.follower;
        t = negabot.turret;
        intake = negabot.intake;

        t.resetEncoder();
        Robot.ALLIANCE = Robot.Alliance.BLUE;

        // Create once and schedule to run continuously
        ppTrack = new PPTracking(t, d, Robot.Alliance.BLUE);

        path(f);

        waitForStart();

        negabot.schedule(

                ppTrack,
                new InstantCommand(() -> { ppTrack.decDeg();}),

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
                        shotPrep( Path9, 0.1),

                        new WaitCommand(500),
                        shoot(),

                        artifactCycle(),
                        artifactCycle()

/*
                        new InstantCommand(() -> { negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path5, true),

                        shotPrep(Path6, 0.1),
                        new WaitCommand(700),
                        shoot()

 */
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

    private Command artifactCycle() {

        return new SequentialCommandGroup(

                new InstantCommand(() -> {
                    negabot.intake.setSpeed(-1);
                }),

                // Drive to artifact pile
                new ArtifactSeekCommand(negabot),

                // Build return path from CURRENT pose
                new InstantCommand(() -> {

                    Pose currentPose = f.getPose();

                    PathChain returnPath =

                            f.pathBuilder()

                                    .addPath(
                                            new BezierLine(
                                                    currentPose,
                                                    new Pose(
                                                            55.886,
                                                            18.599,
                                                            0
                                                    )
                                            )
                                    )

                                    .setLinearHeadingInterpolation(
                                            currentPose.getHeading(),
                                            0
                                    )

                                    .build();

                    f.followPath(returnPath, 1.0, true);
                }),

                // Wait for drive back
                new WaitCommand(1800),

                new InstantCommand(() -> {
                    intake.setSpeed(0);
                }),

                new InstantCommand(() -> {
                    negabot.shooter.setMagazineCover(COVER_OPEN);
                }),

                new WaitCommand(500),

                shoot()
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
                        IntakeColorProcessing.Zone.LEFT, new SequentialCommandGroup(
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
                        IntakeColorProcessing.Zone.RIGHT, new SequentialCommandGroup(
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
                                new Pose(56.935, 7.117),
                                new Pose(55.795, 18.379)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.795, 18.379),
                                new Pose(42.853, 35.735)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.853, 35.735),
                                new Pose(11.375, 35.801)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.375, 35.801),
                                new Pose(55.886, 18.599)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ZoneAGo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.886, 18.599),
                                new Pose(38.416, 5.943),
                                new Pose(12.221, 7.880)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ZoneABack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.221, 7.880),
                                new Pose(55.640, 18.681)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ZoneBGo = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.640, 18.681),
                                new Pose(11.410, 17.726)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ZoneBBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.410, 17.726),
                                new Pose(55.590, 18.612)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        ZoneCGo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.590, 18.612),
                                new Pose(50.720, 44.647),
                                new Pose(11.118, 39.066)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        ZoneCBack = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.118, 39.066),

                                new Pose(55.666, 18.637)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.666, 18.637),
                                new Pose(4.696, 11.957),
                                new Pose(14.882, 21.907),
                                new Pose(11.413, 38.924)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.413, 38.924),
                                new Pose(55.723, 18.625)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.795, 18.379),
                                new Pose(11.217, 8.403)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.052, 7.098),
                                new Pose(11.217, 17.403)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.217, 17.403),
                                new Pose(55.795, 18.379)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.217, 8.403),
                                new Pose(55.795, 18.379)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}