package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Drives straight to a fixed (x, y) from wherever the follower actually is when this command
 * starts, without imposing any heading requirement -- the robot just keeps whatever heading it's
 * already facing for the whole move (start and end heading are both the live heading at
 * initialize() time).
 */
public class DriveToXYCommand extends CommandBase {
    private final Follower follower;
    private final double targetX;
    private final double targetY;
    private final boolean holdEnd;
    private final double maxPower;

    public DriveToXYCommand(Follower follower, double targetX, double targetY, boolean holdEnd) {
        this(follower, targetX, targetY, holdEnd, 1.0);
    }

    public DriveToXYCommand(Follower follower, double targetX, double targetY, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.targetX = targetX;
        this.targetY = targetY;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        Pose live = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(live.getX(), live.getY()), new Pose(targetX, targetY)))
                .setLinearHeadingInterpolation(live.getHeading(), live.getHeading())
                .build();
        follower.followPath(path, maxPower, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
