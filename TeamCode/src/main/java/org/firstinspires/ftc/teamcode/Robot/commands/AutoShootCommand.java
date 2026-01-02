package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class AutoShootCommand extends CommandBase {


    private final Intake intake;
    private final Shooter shooter;
    private final double COVER_OPEN = 0.1;
    private final double COVER_CLOSE = 0.9;
    private final Wait wait;

    public AutoShootCommand(Intake intake, Shooter shooter, Wait wait) {
        this.intake = intake;
        this.shooter = shooter;
        this.wait = wait;
        addRequirements(intake, shooter, wait);
    }

    @Override
    public void execute() {
        shooter.setMagazineCover(COVER_OPEN);
        new WaitCommand(wait, 1);
        intake.setSpeed(-0.75);
        new WaitCommand(wait, 1);
        shooter.setMagazineCover(COVER_CLOSE);
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
