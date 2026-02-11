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
public class ZayansAuto extends CommandOpMode {
    Robot negabot;
    PathChain pre, r1, r1b, r2, r2b, r3, r3b, gate, gate1, gate2, gate3, gateB, gateB0, gateB1;
    Pose startPose = new Pose(26.241, 133.326, Math.toRadians(54));
    Pose shoot = new Pose(58.49942594718714, 84.61882893226179, Math.toRadians(24));
    Pose gateIntake = new Pose(16.624, 63.5, Math.toRadians(-25));
    Pose gateIntake1 = new Pose(16, 63, Math.toRadians(-25));
    Pose gateIntake2 = new Pose(15.624, 63, Math.toRadians(-20));
    Pose gateIntake3 = new Pose(15, 61, Math.toRadians(-25));
    double row1y = 85;
    double row2y = 59;
    double row3y = 35;
    double firstBallx = 42;
    double lastBallx = 20;
    final double COVER_OPEN = 0.1;
    final double COVER_CLOSE = 1.0;
    Intake intake;
    Drive d;
    Follower f;
    Turret t;
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
                new InstantCommand(() -> {negabot.shooter.setVelocity(3700);}),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new SequentialCommandGroup(
                        new InstantCommand(() -> { t.setTargetDeg(15); }),
                        new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                        new FollowPathCommand(f, pre, true),
                        //new WaitCommand(600),
                        shoot(),
                        new FollowPathCommand(f, r2, true),
                        shotPrep(r2b, 18, 0.1),
                        shoot(),
                        new FollowPathCommand(f, gate, true),
                        new WaitCommand(1000),
                        shotPrep(gateB, 23, 0.01),
                        shoot(),
                        new FollowPathCommand(f, r1, true),
                        shotPrep(r1b, 18, 0.01),
                        shoot(),
                        new FollowPathCommand(f, gate1, true),
                        new WaitCommand(1000),
                        shotPrep(gateB, 23, 0.01),
                        shoot(),
                        new FollowPathCommand(f, r3, true),
                        shotPrep(r3b, 23, 0.01),
                        shoot(),
                        new FollowPathCommand(f, gate2, true),
                        new WaitCommand(1000),
                        shotPrep(gateB1, 23, 0.01),
                        shoot()

                        /*
                        new FollowPathCommand(f, gate3, true),
                        new WaitCommand(1000),
                        shotPrep(gateB1, 17, 0.1),
                        shoot()

                         */
                        )

        );
    }
    public Command shoot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                new WaitCommand(500),
                new InstantCommand(() -> {intake.setSpeed(-1);} ),
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
        pre = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(startPose.getX(), startPose.getY()),
                                new Pose(41, 100)
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())
                .build();

        r1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(lastBallx, row1y)
                        )
                ).setLinearHeadingInterpolation(0, Math.toRadians(0))

                .build();

        r1b = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(lastBallx, row1y),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), shoot.getHeading())

                .build();
        r2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(54, 54.490605427974955),
                                new Pose((lastBallx)-2, row2y)
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), Math.toRadians(0), 0.4)

                .build();

        r2b = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose((lastBallx - 2), row2y),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), shoot.getHeading())

                .build();
        r3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(47.4021101992966, 26.24736225087926),
                                new Pose(lastBallx, row3y)
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), Math.toRadians(0), 0.4)

                .build();

        r3b = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(lastBallx, row3y),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), shoot.getHeading())

                .build();
        gate = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(gateIntake.getX(), gateIntake.getY())
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), gateIntake.getHeading())

                .build();
        gate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(gateIntake1.getX(), gateIntake1.getY())
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), gateIntake1.getHeading())

                .build();
        gate2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(gateIntake2.getX(), gateIntake2.getY())
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), gateIntake2.getHeading())

                .build();
        gate3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(gateIntake3.getX(), gateIntake3.getY())
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), gateIntake3.getHeading())

                .build();

        gateB = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(gateIntake.getX(), gateIntake.getY()),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), shoot.getHeading())
                .build();
        gateB0 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(gateIntake.getX(), gateIntake.getY()),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), 0)
                .build();
        gateB1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(gateIntake.getX(), gateIntake.getY()),
                                new Pose(shoot.getX(), (shoot.getY())+5)
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), shoot.getHeading())
                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
        Robot.LAST_TURRET_DEG = negabot.turret.getAngleDeg();
    }
}

