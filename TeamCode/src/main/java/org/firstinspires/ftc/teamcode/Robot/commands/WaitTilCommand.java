package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Wait;

public class WaitTilCommand extends CommandBase {

    private final Shooter pos;
    private final double v, t;
    public WaitTilCommand(Shooter pos, double v, double t) {
        this.pos=pos;
        this.v = v;
        this.t = t;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(pos.getVelocity() - v) <= t;
    }

}
