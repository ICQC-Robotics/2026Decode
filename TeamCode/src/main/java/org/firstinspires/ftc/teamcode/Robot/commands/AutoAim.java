package org.firstinspires.ftc.teamcode.Robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Robot.subsystems.Vision;

public class AutoAim extends CommandBase {
    Vision vision;
    Shooter shooter;

    public AutoAim(Vision vision, Shooter shooter) {
        this.vision = vision;
        this.shooter = shooter;

        addRequirements(vision, shooter);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double ty = vision.getTy();
            double ta = vision.getTa();

            double basePower = 0.5;
            double distanceAdjust = 0.02 * ty;
            double areaAdjust = -0.3 * (ta - 0.1);

            double finalPower = basePower + distanceAdjust + areaAdjust;
            finalPower = Math.max(0.0, Math.min(1.0, finalPower));

            shooter.setPower(finalPower);
        } else {
            shooter.stop();
        }
    }

}
