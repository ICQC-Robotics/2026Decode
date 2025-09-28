package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter extends SubsystemBase {
    private final DcMotorEx rightShooter, leftShooter;


    public Shooter(DcMotorEx rightShooter, DcMotorSimple.Direction rightDir,
                   DcMotorEx leftShooter, DcMotorSimple.Direction leftDir) {

        this.rightShooter = rightShooter;
        this.leftShooter = leftShooter;

        this.rightShooter.setDirection(rightDir);
        this.leftShooter.setDirection(leftDir);

        this.rightShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.rightShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.leftShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        rightShooter.setPower(power);
        leftShooter.setPower(power);
    }

    public void stop() {
        setPower(0.0);
    }

    public void setVelocity(double velocity) {
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightShooter.setVelocity(velocity);
        leftShooter.setVelocity(velocity);
    }

    public double getVelocity() {
        return (rightShooter.getVelocity() + leftShooter.getVelocity()) / 2.0;
    }
}
