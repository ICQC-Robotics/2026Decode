package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PP.Constants;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

public class Drive extends SubsystemBase {
    public final MecanumDrive mD;
    private Pose2d p = new Pose2d(0, 0, 0);
    private final DcMotorEx fR, fL, bR, bL;
    public final Follower follower;
    private final Telemetry telemetry;

    public Drive(HardwareMap h, Telemetry t, Pose startPose,
                 DcMotorEx fR, DcMotorSimple.Direction fRD,
                 DcMotorEx fL, DcMotorSimple.Direction fLD,
                 DcMotorEx bR, DcMotorSimple.Direction bRD,
                 DcMotorEx bL, DcMotorSimple.Direction bLD) {

        this.fR = fR;
        this.fL = fL;
        this.bR = bR;
        this.bL = bL;

        follower = Constants.createFollower(h);
        follower.setStartingPose(startPose);
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
        double drive = g.getLeftY();
        double strafe = g.getLeftX();
        double turn = g.getRightX();

        double fRPower = drive - strafe - turn;
        double fLPower = drive + strafe + turn;
        double bRPower = drive + strafe - turn;
        double bLPower = drive - strafe + turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(fRPower),
                        Math.max(Math.abs(fLPower),
                                Math.max(Math.abs(bRPower), Math.abs(bLPower)))));

        fR.setPower(fRPower / max);
        fL.setPower(fLPower / max);
        bR.setPower(bRPower / max);
        bL.setPower(bLPower / max);
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
