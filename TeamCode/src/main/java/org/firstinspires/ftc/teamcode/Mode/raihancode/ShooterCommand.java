package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ShooterCommand extends CommandBase {
    private final ShooterSystem shooter;
    private final double targetVelocity;

    public ShooterCommand(ShooterSystem shooter, double targetVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTargetVelocity(targetVelocity);
        shooter.update();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTargetVelocity(0);
        shooter.update();
    }

    @Override
    public boolean isFinished() {
        return false; // keeps running until released
    }
}
