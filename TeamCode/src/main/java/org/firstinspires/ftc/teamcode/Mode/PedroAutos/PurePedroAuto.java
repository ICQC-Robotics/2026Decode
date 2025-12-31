package org.firstinspires.ftc.teamcode.Mode.PedroAutos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "JustPedroAuto", group = "Auto")
public class PurePedroAuto extends OpMode {

    private Robot negabot;
    private Follower follower;
    private Paths paths;

    private int pathState = -1;

    private static final Pose START_POSE = new Pose(21.000, 126.000, Math.toRadians(234.6));

    @Override
    public void init() {
        negabot = new Robot(hardwareMap, telemetry);
        follower = negabot.drive.follower;

        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);

        telemetry.addData("Init", "OK");
        telemetry.addData("paths null?", paths == null);
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("pathState", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, false);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, false);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, false);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
    }

    public static class Paths {
        public final PathChain Path1;
        public final PathChain Path2;
        public final PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(21.000, 126.000),
                            new Pose(37.000, 106.000)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(234.6))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(37.000, 106.000),
                            new Pose(42.685, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(234.6), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(42.685, 84.000),
                            new Pose(18.685, 84.000)
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}
