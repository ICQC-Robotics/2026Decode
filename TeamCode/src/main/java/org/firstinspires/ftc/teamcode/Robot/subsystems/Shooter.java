package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter extends SubsystemBase {
    private final DcMotorEx rightShooter, leftShooter;

    public Shooter(DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
                   DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
                   PIDFCoefficients pidf) {

        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;

        this.rightShooter.setDirection(rightDir);
        this.leftShooter.setDirection(leftDir);

        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.rightShooter.setVelocityPIDFCoefficients(
                pidf.p, pidf.i, pidf.d, pidf.f
        );

        this.leftShooter.setVelocityPIDFCoefficients(
                pidf.p, pidf.i, pidf.d, pidf.f
        );
    }

    public void setVelocity(double v) {
        this.rightShooter.setVelocity(v);
        this.leftShooter.setVelocity(v);
    }
}
