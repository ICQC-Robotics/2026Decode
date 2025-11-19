package org.firstinspires.ftc.teamcode.Mode;

import static org.firstinspires.ftc.teamcode.PP.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

import org.firstinspires.ftc.teamcode.Robot.commands.FollowPathCommand;

@Autonomous
public class AutoPedro extends OpMode {
    Path score3;
    PathChain scoop6, score6, scoop9, score9, scoop12, score12;
    Pose startPose, shoot3, grab6, shoot6, grab9, shoot9, grab12, shoot12;

    public void buildPaths() {
        score3 = new Path(new BezierCurve(startPose, shoot3));
        score3.setLinearHeadingInterpolation(startPose.getHeading(), shoot3.getHeading());

        scoop6 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot3, grab6))
                .setLinearHeadingInterpolation(shoot3.getHeading(), grab6.getHeading())
                .build();

        score6 = follower.pathBuilder()
                .addPath(new BezierCurve(grab6, shoot6))
                .setLinearHeadingInterpolation(grab6.getHeading(), shoot6.getHeading())
                .build();

        scoop9 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot6, grab9))
                .setLinearHeadingInterpolation(shoot6.getHeading(), grab9.getHeading())
                .build();

        score9 = follower.pathBuilder()
                .addPath(new BezierCurve(grab9, shoot9))
                .setLinearHeadingInterpolation(grab9.getHeading(), shoot9.getHeading())
                .build();

        scoop12 = follower.pathBuilder()
                .addPath(new BezierCurve(shoot9, grab12))
                .setLinearHeadingInterpolation(shoot9.getHeading(), grab12.getHeading())
                .build();

        score12 = follower.pathBuilder()
                .addPath(new BezierCurve(grab12, shoot12))
                .setLinearHeadingInterpolation(grab12.getHeading(), shoot12.getHeading())
                .build();
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new FollowPathCommand(follower,
                        scoop6
                ),
                new FollowPathCommand(follower,
                        score6
                ),
                new FollowPathCommand(follower,
                        scoop9
                ),
                new FollowPathCommand(follower,
                        score9
                ),
                new FollowPathCommand(follower,
                        scoop12
                ),
                new FollowPathCommand(follower,
                        score12
                )
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
