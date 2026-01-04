package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class WaitToShoot extends CommandBase {
    private Intake intake;
    private Shooter shooter;
    private double time;
    ElapsedTime t;
    double start;

    public WaitToShoot(Intake intake, Shooter shooter, double time) {
        this.intake = intake;
        this.shooter = shooter;
        this.time = time;
        t = new ElapsedTime();
        start = t.milliseconds();
        addRequirements(intake);
    }
    @Override
    public void execute(){
        if(shooter.getVelocity() > shooter.getTargetVelocity() - 100) intake.setSpeed(-1);
        else intake.setSpeed(0);
    }
    @Override
    public boolean isFinished(){
        return t.milliseconds() - start > time;
    }
    @Override
    public void end(boolean interrupted){
        intake.setSpeed(-1);
    }


}
