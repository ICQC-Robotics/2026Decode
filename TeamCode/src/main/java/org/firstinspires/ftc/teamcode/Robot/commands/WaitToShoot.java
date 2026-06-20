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
    private double timeoutMs;
    ElapsedTime t;
    double start;

    public WaitToShoot(Intake intake, Shooter shooter, double timeoutMs) {
        this.intake = intake;
        this.shooter = shooter;
        this.timeoutMs = timeoutMs;
        t = new ElapsedTime();
        start = t.milliseconds();
        addRequirements(intake);
    }

    private boolean isSpunUp() {
        return shooter.getVelocity() > shooter.getTargetVelocity() - 100;
    }

    @Override
    public void execute(){
        intake.setSpeed(isSpunUp() ? -1 : 0);
    }
    @Override
    public boolean isFinished(){
        // Shoot as soon as the shooter's actually spun up, instead of a fixed wait -- timeoutMs
        // is just a safety cap in case it never gets there.
        return isSpunUp() || t.milliseconds() - start > timeoutMs;
    }
    @Override
    public void end(boolean interrupted){
        intake.setSpeed(-1);
    }


}