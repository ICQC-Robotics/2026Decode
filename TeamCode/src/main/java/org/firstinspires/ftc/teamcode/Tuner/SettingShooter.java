package org.firstinspires.ftc.teamcode.Tuner;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SettingShooter extends OpMode {


    private  DcMotorEx rightShooter, leftShooter;
    public  Servo leftCover, rightCover;
    private  PIDFCoefficients pidf;
    private double targetVelocityRPM;
    private double rpmTolerance = 10;
    private double fullPower = 1.0;
    private double offPower  = 0.0;
    private double tbhGain = .001;
    private double currentTbhPower = 0.0;
    private double tbhVal = 0.0;
    private double prevError = 0.0;
    public void init() {
        this.rightShooter = hardwareMap.get(DcMotorEx.class, "shooter1");
        this.leftShooter = hardwareMap.get(DcMotorEx.class, "shooter2");
        this.leftCover = hardwareMap.get(Servo.class, "servo0");
        this.rightCover = hardwareMap.get(Servo.class, "servo2");
        this.pidf =         new PIDFCoefficients(6, 0.0, 0.005, 13.5);
        this.rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
        this.setMagazineCover(1);
        targetVelocityRPM = 1000;
    }
    public void setPIDF(double p, double i, double d, double f) {
        this.pidf.p = p;
        this.pidf.i = i;
        this.pidf.d = d;
        this.pidf.f = f;
        this.rightShooter.setVelocityPIDFCoefficients(p, i, d, f);
        this.leftShooter.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public void setVelocity(double rpm) {
        targetVelocityRPM = rpm;
        if (rpm <= 0) {
            stop();
        }
    }
    public void stop() {
        targetVelocityRPM = 0;
        currentTbhPower = 0;
        tbhVal = 0;
        prevError = 0;
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }
    public void setTBHGain(double gain) {
        this.tbhGain = gain;
    }
    public double getVelocity() {
        double velocityInTPS = this.rightShooter.getVelocity();
        double ticksPerRev = 28;
        return (velocityInTPS * 60.0) / ticksPerRev;
    }

    @Override
    public void loop() {
        if (targetVelocityRPM <= 0) return;
        double currentRPM = getVelocity();
        double error = targetVelocityRPM - currentRPM;
        if (Math.abs(error) > rpmTolerance) {
            if (error > 0) {
                rightShooter.setPower(fullPower);
                leftShooter.setPower(fullPower);
            } else {
                rightShooter.setPower(offPower);
                leftShooter.setPower(offPower);
            }
            currentTbhPower = (error > 0) ? fullPower : offPower;
        } else {
            currentTbhPower += (error * tbhGain);
            currentTbhPower = Range.clip(currentTbhPower, 0.0, 1.0);
            if (Math.signum(error) != Math.signum(prevError)) {
                currentTbhPower = 0.5 * (currentTbhPower + tbhVal);
                tbhVal = currentTbhPower;
            }
            rightShooter.setPower(currentTbhPower);
            leftShooter.setPower(currentTbhPower);
        }
        prevError = error;
    }
    public void setMagazineCover(double pos) {
        leftCover.setPosition(pos);
        rightCover.setPosition(1 - pos);
    }
}
