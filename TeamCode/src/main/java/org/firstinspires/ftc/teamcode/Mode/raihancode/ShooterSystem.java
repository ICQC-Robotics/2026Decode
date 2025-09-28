package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShooterSystem extends SubsystemBase {
    private DcMotorEx shooter;


    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double kF = 0.0;

    double targetVelocity = 0.0;
    double lastError = 0.0;

    public ShooterSystem(DcMotorEx shooterMotor) {
        shooter = shooterMotor;
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void update() {
        double currentVelocity = shooter.getVelocity();
        double error = targetVelocity - currentVelocity;


        double output = (kP * error) + (kF * targetVelocity);

        shooter.setPower(output);

        lastError = error;
    }
}
