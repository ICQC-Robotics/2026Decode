package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class StallTimeoutCommand extends CommandBase {
    private final Follower follower;
    private final long stallMs;
    private Pose lastPose;
    private long lastMoveTime;

    public StallTimeoutCommand(Follower follower, long stallMs) {
        this.follower = follower;
        this.stallMs = stallMs;
    }

    @Override
    public void initialize() {
        lastPose = follower.getPose();
        lastMoveTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        Pose cur = follower.getPose();
        double dist = Math.hypot(cur.getX() - lastPose.getX(), cur.getY() - lastPose.getY());
        if (dist > 1.0) {
            lastPose = cur;
            lastMoveTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - lastMoveTime > stallMs;
    }
}
