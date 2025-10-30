package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class WaitCommand extends CommandBase {

    private final Wait arm;
    private final double pos;

    public WaitCommand(Wait arm, double pos) {
        this.arm=arm;
        this.pos=pos;
        this.arm.start();
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.arm.start();
    }
    @Override
    public boolean isFinished(){
        return arm.elapsed() > pos;
    }

}
