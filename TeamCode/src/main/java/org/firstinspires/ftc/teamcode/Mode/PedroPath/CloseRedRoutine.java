package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close Red Pedro", group = "Autonomous")
@Configurable
public class CloseRedRoutine extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // MUST match Path1 start
        follower.setStartingPose(new Pose(121.321, 119.478, Math.toRadians(90)));

        paths = new Paths(follower);

        pathState = 0;
        follower.followPath(paths.Path1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ================= PATHS =================

    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(121.321, 119.478), new Pose(99.380, 94.955)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(99.380, 94.955), new Pose(99.196, 84.446)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(99.196, 84.446), new Pose(134.044, 83.892)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.044, 83.892), new Pose(99.012, 84.077)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(99.012, 84.077), new Pose(99.749, 59.186)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(99.749, 59.186), new Pose(132.569, 59.370)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(132.569, 59.370), new Pose(99.749, 59.001)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(99.749, 59.001), new Pose(99.933, 34.110)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(99.933, 34.110), new Pose(134.965, 34.848)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(134.965, 34.848), new Pose(100.302, 34.110)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    // ================= STATE MACHINE =================

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) { follower.followPath(paths.Path2); pathState = 1; } break;
            case 1:
                if (!follower.isBusy()) { follower.followPath(paths.Path3); pathState = 2; } break;
            case 2:
                if (!follower.isBusy()) { follower.followPath(paths.Path4); pathState = 3; } break;
            case 3:
                if (!follower.isBusy()) { follower.followPath(paths.Path5); pathState = 4; } break;
            case 4:
                if (!follower.isBusy()) { follower.followPath(paths.Path6); pathState = 5; } break;
            case 5:
                if (!follower.isBusy()) { follower.followPath(paths.Path7); pathState = 6; } break;
            case 6:
                if (!follower.isBusy()) { follower.followPath(paths.Path8); pathState = 7; } break;
            case 7:
                if (!follower.isBusy()) { follower.followPath(paths.Path9); pathState = 8; } break;
            case 8:
                if (!follower.isBusy()) { follower.followPath(paths.Path10); pathState = 9; } break;
            case 9:
                // Done
                break;
        }

        return pathState;
    }
}
