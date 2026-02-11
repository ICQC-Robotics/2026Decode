package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {

    private final DcMotorEx rightShooter, leftShooter;
    public final Servo leftCover, rightCover;
    private final PIDFCoefficients pidf;

    private double targetTPS;
    private double tpsTolerance = 100.0;
    private double fullPower = 1;
    private double offPower = 0.0;
    public Shooter(
            DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
            DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
            Servo leftCover, Servo rightCover, Servo hood,
            PIDFCoefficients pidf
    ) {
        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.leftCover = leftCover;
        this.rightCover = rightCover;
        this.pidf = pidf;

        this.rightShooter.setDirection(rightDir);
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.leftShooter.setDirection(leftDir);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);

        this.setMagazineCover(1);
        targetTPS = 0;
    }
    public double getTargetVelocity() {
        return targetTPS;
    }
    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;

        this.rightShooter.setVelocityPIDFCoefficients(p, i, d, f);
        this.leftShooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setMagazineCover(double pos) {
        leftCover.setPosition(pos);
        rightCover.setPosition(1 - pos);
    }
    public void setVelocity(double tps) {
        targetTPS = tps;
        if (tps <= 0) {
            stop();
        }
    }

    public void stop() {
        targetTPS = 0;
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    public double getVelocity() {
        return this.rightShooter.getVelocity();
    }

    @Override
    public void periodic() {
        if (getVelocity() < (targetTPS - tpsTolerance)) {
            rightShooter.setPower(fullPower);
            leftShooter.setPower(fullPower);
        }
        else if (getVelocity() > (targetTPS + tpsTolerance)) {
            rightShooter.setPower(offPower);
            leftShooter.setPower(offPower);
        }
        else {
            rightShooter.setVelocity(targetTPS);
            leftShooter.setVelocity(targetTPS);
        }
    }
}