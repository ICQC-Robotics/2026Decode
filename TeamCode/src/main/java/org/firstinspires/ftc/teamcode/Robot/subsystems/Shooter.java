package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {
    private DcMotorEx rightShooter, leftShooter;
    private final Servo cover;
    private PIDFCoefficients pidf;

    public Shooter(DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
                   Servo cover, PIDFCoefficients pidf) {

        this.rightShooter = rightShooter;
        this.cover = cover;
        this.pidf = pidf;

        this.rightShooter.setDirection(rightDir);
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.rightShooter.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);
    }


    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;

        this.rightShooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setMagazineCover(double pos) {
        cover.setPosition(pos);
    }

    public void setVelocity(double v) {
        double ticksPerRev = rightShooter.getMotorType().getTicksPerRev();
        double ticksPerSec = (v / 60.0) * ticksPerRev;
        rightShooter.setVelocity(ticksPerSec);
    }

    public double getVelocity() {
        return this.rightShooter.getVelocity();
    }
}
