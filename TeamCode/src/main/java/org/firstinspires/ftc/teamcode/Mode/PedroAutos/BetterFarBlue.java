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
import org.firstinspires.ftc.teamcode.Robot.commands.ShooterStandBy;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

@Autonomous
public class BetterFarBlue extends CommandOpMode {
    Robot negabot;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
    Pose startPose = new Pose(57.05219206680585, 7.098121085594997, Math.toRadians(0));
    final double COVER_OPEN = 0.1;
    final double COVER_CLOSE = 1.0;

    Drive d;
    Follower f;
    Turret t;

    Intake intake;


    @Override
    public void initialize() {
        negabot = new Robot(hardwareMap, telemetry, startPose);

        d = negabot.drive;
        f = d.follower;
        t = negabot.turret;
        intake = negabot.intake;

        t.resetEncoder();

        path(f);

        waitForStart();
        negabot.schedule(
                new InstantCommand(() -> { negabot.shooter.setVelocity(4150);}),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {negabot.turret.setTargetDeg(20); }),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN); }),
                        new WaitCommand(3200),
                        shoot(),
                        new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path1, true),
                        new FollowPathCommand(f, Path2),
                        new FollowPathCommand(f, Path3, true),
                        shotPrep(Path4, 15),
                        new WaitCommand(1000),
                        shoot(),
                        gateIntake(15),
                        gateIntake(15),
                        gateIntake2(18),
                        gateIntake2(18),
                        new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path5, true)
                )
        );

    }
    public Command gateIntake(int a){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                new FollowPathCommand(f, Path1, true),
                shotPrep(Path4, a),
                new WaitCommand(500),
                shoot()
        );
    }
    public Command gateIntake2(int a){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                new FollowPathCommand(f, Path2, true),
                shotPrep(Path3, a),
                new WaitCommand(500),
                shoot()
        );
    }


    public Command shoot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1);} ),
                new WaitCommand(500),
                new InstantCommand(() -> {negabot.intake.setSpeed(0);} ),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} )
        );
    }
    public Command shotPrep(PathChain path, double turretAngle){
        return new ParallelCommandGroup(

                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> {intake.setSpeed(0);} ),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);}
                        )),
                new InstantCommand(() -> { t.setTargetDeg(turretAngle); })

        );
    }

    public void path(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.052, 7.098),

                                new Pose(9.217, 8.403)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.052, 7.098),

                                new Pose(9.217, 17.403)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.217, 17.403),

                                new Pose(57.052, 7.098)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

//        Path2 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(9.217, 8.403),
//                                new Pose(26.329, 13.675),
//                                new Pose(23.497, 8.248)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                .build();

//        Path3 = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(23.497, 8.248),
//
//                                new Pose(9.217, 8.403)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.217, 8.403),

                                new Pose(57.052, 7.098)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.052, 7.098),

                                new Pose(38, 18.403)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.230, 35.649),

                                new Pose(11.762, 36.142)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.762, 36.142),

                                new Pose(60.568, 13.315)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.568, 13.315),

                                new Pose(46.756, 59.956)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(46.756, 59.956),

                                new Pose(11.447, 59.720)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.447, 59.720),

                                new Pose(60.418, 13.196)
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
