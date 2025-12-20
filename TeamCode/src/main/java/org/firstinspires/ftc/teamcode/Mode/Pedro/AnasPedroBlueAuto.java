package org.firstinspires.ftc.teamcode.Mode.Pedro;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;


class Paths {


}
@Autonomous(group = "!",name = "blue chopped istg")
public class AnasPedroBlueAuto extends OpMode {
    private Follower follower;
    private Robot negabot;
    Path s;
    PathChain i1, s1, i2, s2, i3, s3, leave;
    Pose startPose, shoot, grab1i, grab1f, grab2i, grab2f, grab3i, grab3f, leavePos;
    double shooterVelo = 2150; //speed the shooter should shoot at
    double shooterWaitTime = 2; //how long (in seconds) the shooter should shoot for
    double robotFrontToCenter = 8; //distance between the front of the robot and the center
    double intakeXi = 100; //X to start intaking at
    double intakeHeadingRad = Math.toRadians(180);
    double intakeXf = 120; //X to stop intaking at
    double r1y = 84; //Y of top row
    double r2y = 60; //Y of middle row
    double r3y = 36; //Y of bottom row
    private Paths paths;
    private SequentialCommandGroup autoSequence;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;

    public void buildPaths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.571, 18.083))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.571, 18.083),
                                new Pose(70.507, 82.783),
                                new Pose(40.147, 109.825)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(40.147, 109.825),
                                new Pose(47.281, 78.802),
                                new Pose(12.276, 83.613)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.276, 83.613),
                                new Pose(25.217, 54.083),
                                new Pose(77.309, 77.641),
                                new Pose(39.982, 109.991)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(39.982, 109.991),
                                new Pose(75.816, 55.742),
                                new Pose(12.442, 59.724)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.442, 59.724),
                                new Pose(80.793, 50.765),
                                new Pose(39.982, 109.991)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }


    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;
        buildPaths(follower);
        follower.setStartingPose(new Pose(56.000, 8.000));
    }

    public void start(){
        // build a sequential group of commands
        autoSequence = new SequentialCommandGroup(
                new InstantCommand(()->{
                    negabot.shooter.setVelocity(shooterVelo);
                }),
                new InstantCommand(()->{
                    negabot.intake.setSpeed(-1);
                }),
                // Path 1
                new FollowPathCommand(follower, Path1),
                // Path 2
                new FollowPathCommand(follower, Path2),
                new InstantCommand(() ->{
                    negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                }),
                new WaitCommand(negabot.wait, shooterWaitTime),
                new InstantCommand(() ->{
                    negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                }),

                // Path 3 (reversed path)
                new FollowPathCommand(follower, Path3),
                // Path 4
                new FollowPathCommand(follower, Path4),
                new FollowPathCommand(follower, Path2),
                new InstantCommand(() ->{
                    negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                }),
                new WaitCommand(negabot.wait, shooterWaitTime),
                new InstantCommand(() ->{
                    negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                }),

                // Path 5 (reversed path)
                new FollowPathCommand(follower, Path5),
                // Path 6
                new FollowPathCommand(follower, Path6),
                new FollowPathCommand(follower, Path2),
                new InstantCommand(() ->{
                    negabot.shooter.setMagazineCover(AutoAim.Positions.OPEN_COVER.getPos());
                }),
                new WaitCommand(negabot.wait, shooterWaitTime),
                new InstantCommand(() ->{
                    negabot.shooter.setMagazineCover(AutoAim.Positions.CLOSED_COVER.getPos());
                })


        );

        autoSequence.initialize();  // prepare all commands
        CommandScheduler.getInstance().schedule(autoSequence);

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
