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
public class BetterFarRed extends CommandOpMode {
    Robot negabot;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
    Pose startPose = new Pose(mirrorX(57.05219206680585), 6.598121085594997, mirrorHeadingRad(Math.toRadians(0)));
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
                new InstantCommand(() -> { negabot.shooter.setVelocity(4000);}),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new SequentialCommandGroup(
                        new InstantCommand(() -> {negabot.turret.setTargetDeg(mirrorTurretDeg(20)); }),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN); }),
                        new WaitCommand(3200),
                        shoot(),
                        new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path1, true),
                        new FollowPathCommand(f, Path2),
                        new FollowPathCommand(f, Path3, true),
                        shotPrep(Path4, mirrorTurretDeg(17), 0.1),
                        new WaitCommand(1000),
                        shoot(),
                        gateIntake(mirrorTurretDeg(17)),
                        gateIntake(mirrorTurretDeg(17)),
                        gateIntake2(mirrorTurretDeg(20)),
                        gateIntake2(mirrorTurretDeg(20)),
                        new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                        new FollowPathCommand(f, Path5, true)
                )
        );

    }
    public Command gateIntake(double a){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                new FollowPathCommand(f, Path1, true),
                shotPrep(Path4, a, 0.1),
                new WaitCommand(500),
                shoot()
        );
    }
    public Command gateIntake2(double a){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.intake.setSpeed(-1); }),
                new FollowPathCommand(f, Path2, true),
                shotPrep(Path3, a, 0.1),
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
    public Command shotPrep(PathChain path, double turretAngle, double hoodPos){
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
                                new Pose(mirrorX(57.052), 7.098),

                                new Pose(mirrorX(9.217), 8.403)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

                .build();
        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(57.052), 7.098),

                                new Pose(mirrorX(9.217), 17.403)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

                .build();
        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(9.217), 17.403),

                                new Pose(mirrorX(57.052), 7.098)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

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
                                new Pose(mirrorX(9.217), 8.403),

                                new Pose(mirrorX(57.052), 7.098)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(mirrorX(57.052), 7.098),

                                new Pose(mirrorX(38), 18.403)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(43.230, 35.649),

                                new Pose(11.762, 36.142)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

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
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.447, 59.720),

                                new Pose(60.418, 13.196)
                        )
                ).setLinearHeadingInterpolation(mirrorHeadingRad(Math.toRadians(0)), mirrorHeadingRad(Math.toRadians(0)))

                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
    private static double mirrorX(double x) {
        // Mirror across x = 72 => x' = 144 - x
        return 144.0 - x;
    }

    private static double mirrorHeadingRad(double headingRad) {
        // Mirror across vertical axis => theta' = pi - theta
        double out = Math.PI - headingRad;
        while (out < 0) out += 2 * Math.PI;
        while (out >= 2 * Math.PI) out -= 2 * Math.PI;
        return out;
    }

    private static double mirrorTurretDeg(double turretDeg) {
        // 135° is straight forward, so mirror around 135:
        // turret' = 270 - turret
        double out = 270.0 - turretDeg;
        while (out < 0) out += 360.0;
        while (out >= 360.0) out -= 360.0;
        return out;
    }

}
