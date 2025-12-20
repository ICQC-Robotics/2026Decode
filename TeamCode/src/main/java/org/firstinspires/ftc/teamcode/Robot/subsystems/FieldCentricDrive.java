package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PP.Constants;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

public class FieldCentricDrive extends SubsystemBase {
    public final MecanumDrive mD;
    private Pose2d p = new Pose2d(0, 0, 0);
    private final DcMotorEx fR, fL, bR, bL;
    public final Follower follower;
    private final Telemetry telemetry;

    public FieldCentricDrive(HardwareMap h, Telemetry t,
                             DcMotorEx fR, DcMotorSimple.Direction fRD,
                             DcMotorEx fL, DcMotorSimple.Direction fLD,
                             DcMotorEx bR, DcMotorSimple.Direction bRD,
                             DcMotorEx bL, DcMotorSimple.Direction bLD) {

        this.fR = fR;
        this.fL = fL;
        this.bR = bR;
        this.bL = bL;

        follower = Constants.createFollower(h);
        telemetry = t;

        fR.setDirection(fRD);
        fL.setDirection(fLD);
        bR.setDirection(bRD);
        bL.setDirection(bLD);

        fR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        mD = new MecanumDrive(h, p);
    }

    public void movement(GamepadEx g) {


        double x = g.getLeftX();
        double y = g.getLeftY();

        double heading = this.getHeading();


        double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double frontLeft = rotatedY + rotatedX + g.getRightX();
        double frontRight = rotatedY - rotatedX - g.getRightX();
        double backLeft = rotatedY - rotatedX + g.getRightX();
        double backRight = rotatedY + rotatedX - g.getRightX();


        double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight),
                Math.max(Math.abs(backLeft), Math.abs(backRight))));
        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        fL.setPower(frontLeft);
        fR.setPower(frontRight);
        bL.setPower(backLeft);
        bR.setPower(backRight);

    }

    public MecanumDrive getMecanumDrive() {
        return mD;
    }
    public DcMotorEx getFr() { return fR; }
    public DcMotorEx getFl() { return fL; }
    public DcMotorEx getBr() { return bR; }
    public DcMotorEx getBl() { return bL; }

    public double getX() {
        return follower.getPose().getX();
    }

    public double getY() {
        return follower.getPose().getY();
    }

    public double getHeading() {
        return follower.getPose().getHeading();
    }

    public void turnInPlace(double turnPower) {
        fL.setPower(turnPower);
        bL.setPower(turnPower);
        fR.setPower(-turnPower);
        bR.setPower(-turnPower);
    }

    public void stop() {
        fL.setPower(0);
        bL.setPower(0);
        fR.setPower(0);
        bR.setPower(0);
    }

    @Override
    public void periodic() {
        follower.update();
        mD.localizer.update();

        telemetry.addData("X", this.getX());
        telemetry.addData("Y", this.getY());
        telemetry.addData("Heading", this.getHeading());
        telemetry.update();
    }
}
