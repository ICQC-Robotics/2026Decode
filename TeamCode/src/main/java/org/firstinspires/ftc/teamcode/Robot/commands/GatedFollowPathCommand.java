package org.firstinspires.ftc.teamcode.Robot.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.util.function.BooleanSupplier;

/**
 * A {@link FollowPathCommand} that refuses to <em>begin</em> a new drivetrain path once a cutoff
 * condition is hit -- used by the close autos to stop launching new path-follows late in the auto
 * period (e.g. after 28.5 s of a 30 s auto).
 *
 * <p>When the gate is closed at {@link #initialize()}, the command does not call
 * {@code followPath(...)}, fires the {@code onSkip} callback (the close autos use this to end the
 * whole OpMode), and reports finished immediately. A path that is already running is unaffected;
 * only the start of a new one is gated.
 */
public class GatedFollowPathCommand extends FollowPathCommand {
    private final BooleanSupplier canStart;
    private final Runnable onSkip;
    private boolean skipped = false;

    public GatedFollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd,
                                  BooleanSupplier canStart, Runnable onSkip) {
        super(follower, pathChain, holdEnd);
        this.canStart = canStart;
        this.onSkip = onSkip;
    }

    @Override
    public void initialize() {
        if (!canStart.getAsBoolean()) {
            skipped = true;
            if (onSkip != null) onSkip.run();
            return;
        }
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return skipped || super.isFinished();
    }
}
