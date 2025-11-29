package org.firstinspires.ftc.teamcode.Mode;

import static org.firstinspires.ftc.teamcode.PP.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PP.Constants;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoAim;
import org.firstinspires.ftc.teamcode.Robot.commands.AutoIntake;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Robot.commands.WaitCommand;

@Autonomous
public class AutoPedro extends OpMode {
    private Follower follower;
    Path s;
    PathChain i1, s1, i2, s2, i3, s3, leave;
    Pose startPose, shoot, grab1i, grab1f, grab2i, grab2f, grab3i, grab3f, leavePos;
    double shooterVelo = 1500; //speed the shooter should shoot at TODO: Find
    double shooterWaitTime = 1; //how long (in seconds) the shooter should shoot for TODO: Find
    double robotFrontToCenter = 6.7; //distance between the front of the robot and the center TODO: Fimd
    double intakeXi = 100; //X to start intaking at
    double intakeXf = 120; //X to stop intaking at
    double r1y = 84; //Y of top row
    double r2y = 60; //Y of middle row
    double r3y = 36; //Y of bottom row

    public void buildPaths() {
        startPose = new Pose(144 - 14.57 - robotFrontToCenter * Math.cos(Math.toRadians(54.046)), 144 - 15.6 - robotFrontToCenter * Math.sin(Math.toRadians(54.046)), Math.toRadians(54.046));
        shoot = new Pose(96, 96, Math.toRadians(54.046));
        grab1i = new Pose(intakeXi, r1y, 0);
        grab1f = new Pose(intakeXf, r1y, 0);
        grab2i = new Pose(intakeXi, r2y, 0);
        grab2f = new Pose(intakeXf, r2y, 0);
        grab3i = new Pose(intakeXi, r3y, 0);
        grab3f = new Pose(intakeXf, r3y, 0);
        leavePos = new Pose(110, 100, Math.toRadians(54.046));

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
        Robot negabot = new Robot(hardwareMap, null, null);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new FollowPathCommand(follower, s),
                            new InstantCommand(() -> {
                                negabot.shooter.setVelocity(shooterVelo);
                            }),
                            new InstantCommand(() -> {
                                negabot.intake.setSpeed(-1);
                            })
                    ),
                    new InstantCommand(() -> {
                        negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                    }),
                    new WaitCommand(negabot.wait, shooterWaitTime),
                    new ParallelCommandGroup(
                            new FollowPathCommand(follower, i1),
                            new InstantCommand(() -> {
                                negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                            }),
                            new InstantCommand(() -> {
                                negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos());
                            })
                    ),
                    new InstantCommand(() -> {
                        negabot.intake.set(AutoIntake.Positions.UPPER_INTAKE.getPos());
                    }),
                    new FollowPathCommand(follower, s1),
                    new InstantCommand(() -> {
                        negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                    }),
                    new WaitCommand(negabot.wait, shooterWaitTime),
                    new ParallelCommandGroup(
                            new FollowPathCommand(follower, i2),
                            new InstantCommand(() -> {
                                negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                            }),
                            new InstantCommand(() -> {
                                negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos());
                            })
                    ),
                    new InstantCommand(() -> {
                        negabot.intake.set(AutoIntake.Positions.UPPER_INTAKE.getPos());
                    }),
                    new FollowPathCommand(follower, s2),
                    new InstantCommand(() -> {
                        negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                    }),
                    new WaitCommand(negabot.wait, shooterWaitTime),
                    new ParallelCommandGroup(
                            new FollowPathCommand(follower, i3),
                            new InstantCommand(() -> {
                                negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                            }),
                            new InstantCommand(() -> {
                                negabot.intake.set(AutoIntake.Positions.LOWER_INTAKE.getPos());
                            })
                    ),
                    new InstantCommand(() -> {
                        negabot.intake.set(AutoIntake.Positions.UPPER_INTAKE.getPos());
                    }),
                    new FollowPathCommand(follower, s3),
                    new InstantCommand(() -> {
                        negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                    }),
                    new WaitCommand(negabot.wait, shooterWaitTime),
                    new FollowPathCommand(follower, leave)
            )
        );
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
