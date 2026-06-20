package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;

/**
 * Waits until the robot has stopped moving AND the turret is on target, instead of a fixed
 * delay. Bounded by timeoutMs so a noisy localizer or an out-of-range turret target can't hang
 * the routine -- it just gives up and lets the shot go after that.
 */
public class WaitUntilReadyToShoot extends CommandBase {
    private final Follower follower;
    private final Turret turret;
    private final double maxVelocity;
    private final double turretToleranceDeg;
    private final long timeoutMs;
    private long startMs;

    public WaitUntilReadyToShoot(Follower follower, Turret turret,
                                  double maxVelocity, double turretToleranceDeg, long timeoutMs) {
        this.follower = follower;
        this.turret = turret;
        this.maxVelocity = maxVelocity;
        this.turretToleranceDeg = turretToleranceDeg;
        this.timeoutMs = timeoutMs;
    }

    @Override
    public void initialize() {
        startMs = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - startMs >= timeoutMs) return true;

        boolean stopped = follower.getVelocity().getMagnitude() <= maxVelocity;
        boolean aligned = turret.atTarget(turretToleranceDeg);
        return stopped && aligned;
    }
}
