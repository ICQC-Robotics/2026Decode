package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;

//default velocity set when auto aim inactive
public class ShooterStandBy extends CommandBase {
    private final Shooter shooter;
    private static final double STANDBY_VELOCITY = 2300;

    public ShooterStandBy(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPIDF(0.095, 0.0, 0, 0.57 * (STANDBY_VELOCITY / 3000));
    }
    //.49 is the new f found at 12.82v

    @Override
    public void execute() {
        shooter.setVelocity(STANDBY_VELOCITY);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
