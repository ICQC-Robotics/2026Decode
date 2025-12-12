package org.firstinspires.ftc.teamcode.Mode.Pedro;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.PP.Constants;
import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

@Autonomous(name = "Straight Test Auto")
public class AutoPedroStraightLineTest extends OpMode {

    private Follower follower;
    Pose startPose, endPose;
    Path forwardPath;

    public void buildPaths() {
        startPose = new Pose(72, 72, 0);
        endPose = new Pose(120, 72, 0);

        forwardPath = new Path(new BezierLine(startPose, endPose));
        forwardPath.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, forwardPath)
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
