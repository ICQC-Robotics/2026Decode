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

@Autonomous
public class ZayansAuto extends CommandOpMode {
    Robot negabot;
    PathChain pre, r1, r1b, r2, r2b, r3, r3b, gate, gateB;
    Pose startPose = new Pose(26.241, 133.326, Math.toRadians(54));
    Pose shoot = new Pose(58.49942594718714, 84.61882893226179, Math.toRadians(71)); //TODO: ADD
    Pose gateIntake = new Pose(17.624, 88.278, Math.toRadians(-30)); //TODO: ADD
    double row1y = 84; //TODO: ADD
    double row2y = 60; //TODO: ADD
    double row3y = 36; //TODO: ADD
    double firstBallx = 42; //TODO: ADD
    double lastBallx = 25; //TODO: ADD
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

        path(f);

        waitForStart();
        negabot.schedule(
                new ShooterStandBy(negabot.shooter),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE); }),
                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.3); }),
                new SequentialCommandGroup(
                        new InstantCommand(() -> { t.setTargetDeg(0); }),
                        new FollowPathCommand(f, pre, true),
                        shoot(),
                        new FollowPathCommand(f, r2, true),
                        shotPrep(r2b, 50.5, 0.1),
                        shoot(),
                        gateShot(),
                        new FollowPathCommand(f, r1, true),
                        shotPrep(r1b, 73.5, 0.05),
                        shoot(),
                        gateShot(),
                        new FollowPathCommand(f, r3, true),
                        shotPrep(r3b, 73.5, 0.05),
                        shoot(),
                        gateShot())
        );
    }
    public Command shoot(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_OPEN);} ),
                new WaitCommand(1000),
                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                new WaitCommand(1000),
                new InstantCommand(() -> {intake.setSpeed(-1);} ),
                new InstantCommand(() -> {negabot.shooter.setMagazineCover(COVER_CLOSE);} )
        );
    }
    public Command shotPrep(PathChain path, double turretAngle, double hoodPos){
        return new ParallelCommandGroup(
                new FollowPathCommand(f, path, true),
                new InstantCommand(() -> {intake.setSpeed(0);} ),
                new InstantCommand(() -> { t.setTargetDeg(turretAngle); }),
                new InstantCommand(() -> {negabot.shooter.setHoodPos(hoodPos); })
        );
    }
    public Command gateShot(){
        return new SequentialCommandGroup(
                new FollowPathCommand(f, gate, true),
                new InstantCommand(() -> { t.setTargetDeg(73.5); }),
                new InstantCommand(() -> {negabot.shooter.setHoodPos(0.05); }),
                new InstantCommand(() -> {intake.setSpeed(0);} ),
                new FollowPathCommand(f, gateB, true),
                shoot()
        );
    }

    public void path(Follower follower) {
        pre = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(startPose.getX(), startPose.getY()),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading())

                .build();

        r1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(firstBallx, row1y),
                                new Pose(lastBallx, row1y)
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), Math.toRadians(0))

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
                                new Pose(firstBallx, row2y),
                                new Pose(lastBallx, row2y)
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), Math.toRadians(0))

                .build();

        r2b = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(lastBallx, row2y),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), shoot.getHeading())

                .build();
        r3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(shoot.getX(), shoot.getY()),
                                new Pose(firstBallx, row3y),
                                new Pose(lastBallx, row3y)
                        )
                ).setLinearHeadingInterpolation(shoot.getHeading(), Math.toRadians(0))

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

        gateB = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(gateIntake.getX(), gateIntake.getY()),
                                new Pose(shoot.getX(), shoot.getY())
                        )
                ).setLinearHeadingInterpolation(gateIntake.getHeading(), shoot.getHeading())

                .build();
    }

    public void run() {
        super.run();
        Robot.LAST_POSE = negabot.drive.follower.getPose().copy();
    }
}

