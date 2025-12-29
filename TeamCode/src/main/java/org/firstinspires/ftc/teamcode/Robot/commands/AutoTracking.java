package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

public class AutoTracking extends CommandBase {

    private final Turret turret;
    private final Vision vision;

    public AutoTracking(Turret turret, Vision vision) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.holdCurrentAngle();
        turret.setVisionTxDeg(Double.NaN);
    }

    @Override
    public void execute() {
        double tx = vision.getTx();

        if (Double.isNaN(tx)) {
            turret.setVisionTxDeg(Double.NaN);
            return;
        }

        turret.setVisionTxDeg(tx);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setVisionTxDeg(Double.NaN);
        turret.holdCurrentAngle();
    }
}