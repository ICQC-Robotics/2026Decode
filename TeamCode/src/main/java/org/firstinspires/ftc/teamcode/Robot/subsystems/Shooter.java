package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {
    private final DcMotorEx rightShooter, leftShooter;
    private final Servo cover;
    private PIDFCoefficients pidf;

    public Shooter(DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
                   DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
                   Servo cover, PIDFCoefficients pidf) {

        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.cover = cover;
        this.pidf = pidf;

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

    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;
    }

    public void setMagazineCover(double pos) {
        cover.setPosition(pos);
    }

    public void setVelocity(double v) {
        this.rightShooter.setVelocity(v);
        this.leftShooter.setVelocity(v);
    }

    public double getVelocity() {
        return this.rightShooter.getVelocity();
    }
}
