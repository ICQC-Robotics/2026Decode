package org.firstinspires.ftc.teamcode.Mode.Pedro;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitCommand;

@Autonomous(name = "AutoPedroRedFar", group = "Auto")
public class PedroRedFarAuto extends OpMode {
    private Follower follower;
    private Robot negabot;

    Path s;
    PathChain i1, s1, i2, s2, i3, s3, leave;
    Pose startPose, shoot, grab1i, grab1f, grab2i, grab2f, grab3i, grab3f, leavePos;

    double shooterVelo = 2950; // speed the shooter should shoot at
    double shooterWaitTime = 2; // how long (in seconds) the shooter should shoot for
    double intakeXi = 100; // X to start intaking at
    double intakeHeadingRad = Math.toRadians(180);
    double intakeXf = 120; // X to stop intaking at
    double r1y = 84; // Y of top row
    double r2y = 60; // Y of middle row
    double r3y = 36; // Y of bottom row

    public void buildPaths() {
        startPose = new Pose(
                88,
                8,
                Math.toRadians(90)
        );

        shoot = new Pose(
                86,
                10,
                Math.toRadians(70)
        );

        grab1i = new Pose(intakeXi, r3y, intakeHeadingRad);
        grab1f = new Pose(intakeXf, r3y, intakeHeadingRad);

        grab2i = new Pose(intakeXi, r2y, intakeHeadingRad);
        grab2f = new Pose(intakeXf, r2y, intakeHeadingRad);

        grab3i = new Pose(intakeXi, r1y, intakeHeadingRad);
        grab3f = new Pose(intakeXf, r1y, intakeHeadingRad);

        leavePos = new Pose(100, 20, Math.toRadians(70));

        s = new Path(new BezierCurve(startPose, shoot));
        s.setLinearHeadingInterpolation(startPose.getHeading(), shoot.getHeading());

        i1 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, grab1i))
                .addPath(new BezierCurve(grab1i, grab1f))
                .setLinearHeadingInterpolation(shoot.getHeading(), grab1f.getHeading())
                .build();

        s1 = follower.pathBuilder()
                .addPath(new BezierCurve(grab1f, shoot))
                .setLinearHeadingInterpolation(grab1f.getHeading(), shoot.getHeading())
                .build();

        i2 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, grab2i))
                .addPath(new BezierCurve(grab2i, grab2f))
                .setLinearHeadingInterpolation(shoot.getHeading(), grab2f.getHeading())
                .build();

        s2 = follower.pathBuilder()
                .addPath(new BezierCurve(grab2f, shoot))
                .setLinearHeadingInterpolation(grab2f.getHeading(), shoot.getHeading())
                .build();

        i3 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, grab3i))
                .addPath(new BezierCurve(grab3i, grab3f))
                .setLinearHeadingInterpolation(shoot.getHeading(), grab3f.getHeading())
                .build();

        s3 = follower.pathBuilder()
                .addPath(new BezierCurve(grab3f, shoot))
                .setLinearHeadingInterpolation(grab3f.getHeading(), shoot.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, leavePos))
                .setLinearHeadingInterpolation(shoot.getHeading(), leavePos.getHeading())
                .build();
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;

        telemetry.addData("follower", follower);
        telemetry.update();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, s),
                                new InstantCommand(() ->
                                        negabot.shooter.setVelocity(shooterVelo)
                                ),
                                new InstantCommand(() ->
                                        negabot.intake.setSpeed(-1)
                                )
                        ),
                        new InstantCommand(() ->
                                negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos())
                        ),
                        new WaitCommand(negabot.wait, shooterWaitTime),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, i1),
                                new InstantCommand(() ->
                                        negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos())
                                ),
                                new InstantCommand(() ->
                                        negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos())
                                )
                        ),
                        new InstantCommand(() ->
                                negabot.intake.set(AutoIntake.Positions.UPPER_INTAKE.getPos())
                        ),
                        new FollowPathCommand(follower, s1),

                        new InstantCommand(() ->
                                negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos())
                        ),
                        new WaitCommand(negabot.wait, shooterWaitTime),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, i2),
                                new InstantCommand(() ->
                                        negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos())
                                ),
                                new InstantCommand(() ->
                                        negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos())
                                )
                        ),
                        new InstantCommand(() ->
                                negabot.intake.set(AutoIntake.Positions.UPPER_INTAKE.getPos())
                        ),
                        new FollowPathCommand(follower, s2),

                        new InstantCommand(() ->
                                negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos())
                        ),
                        new WaitCommand(negabot.wait, shooterWaitTime),

                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, i3),
                                new InstantCommand(() ->
                                        negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos())
                                ),
                                new InstantCommand(() ->
                                        negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos())
                                )
                        ),
                        new InstantCommand(() ->
                                negabot.intake.set(AutoIntake.Positions.UPPER_INTAKE.getPos())
                        ),
                        new FollowPathCommand(follower, s3),

                        new InstantCommand(() ->
                                negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos())
                        ),
                        new WaitCommand(negabot.wait, shooterWaitTime),

                        new FollowPathCommand(follower, leave)
                )
        );
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }
        CommandScheduler.getInstance().run();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
