package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;

public class ShooterStandBy extends CommandBase {

    private final Shooter shooter;

    public ShooterStandBy(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(AutoAim.MIN_V);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
