package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;

public class HoldCommand extends CommandBase {
    private final Drive drive;
    private final Follower follower;
    PPTracking p;
    Pose holdPose = null;
    GamepadEx g;
    public HoldCommand(Drive drive, GamepadEx g, PPTracking p) {
        this.drive = drive;
        this.follower = drive.follower;
        this.g = g;
        this.p = p;
        follower.update();
        holdPose = follower.getPose();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        follower.update();
        follower.followPath(new Path(new BezierLine(holdPose, holdPose)));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(g.getLeftX()) > 0.1 ||Math.abs(g.getRightX()) > 0.1 ||Math.abs(g.getRightY()) > 0.1 ||Math.abs(g.getLeftY()) > 0.1;
    }
}
