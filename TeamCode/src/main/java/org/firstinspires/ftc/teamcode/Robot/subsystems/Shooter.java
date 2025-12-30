package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {
    private final DcMotorEx rightShooter, leftShooter;
    public final Servo leftCover, rightCover, hood;
    private final PIDFCoefficients pidf;

    public Shooter(DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
                   DcMotorEx leftShooter, DcMotorSimple.Direction leftDir,
                   Servo leftCover, Servo rightCover,Servo hood,
                   PIDFCoefficients pidf) {

        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;
        this.leftCover = leftCover;
        this.rightCover = rightCover;
        this.hood = hood;
        this.pidf = pidf;

        this.rightShooter.setDirection(rightDir);
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftShooter.setDirection(leftDir);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);

        this.setHoodPos(.5); //TODO: find default hood angle pos
        this.setMagazineCover(.24); // TODO: change accordingly to new cover
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
        rightCover.setPosition(1-pos);
    }

    public void setHoodPos(double pos) {
        hood.setPosition(pos);
    }

    public void setVelocity(double rpm) {
        double ticksPerRev = 28;
        double ticksPerSec = (rpm / 60.0) * ticksPerRev;

        rightShooter.setVelocity(ticksPerSec);
        leftShooter.setVelocity(ticksPerSec);

    }

    public double getVelocity() {
        double velocityInTPS = this.rightShooter.getVelocity();
        double ticksPerRev = 28;

        return (velocityInTPS * 60.0) / ticksPerRev;
    }
}
