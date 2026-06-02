package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitToShoot extends CommandBase {
    private final Intake intake;
    private final Shooter shooter;
    private final double timeMs;
    private final ElapsedTime timer = new ElapsedTime();

    public WaitToShoot(Intake intake, Shooter shooter, double time) {
        this.intake = intake;
        this.shooter = shooter;
        this.timeMs = time;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute(){
        if(shooter.isAtTargetVelocity(100)) intake.setSpeed(-1);
        else intake.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return timer.milliseconds() >= timeMs;
    }

    @Override
    public void end(boolean interrupted){
        intake.setSpeed(0);
    }
}
