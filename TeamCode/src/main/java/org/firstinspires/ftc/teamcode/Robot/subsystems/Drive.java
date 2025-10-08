package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RR.PinpointLocalizer;

public class Drive extends SubsystemBase {
    DcMotorEx fR;
    DcMotorEx fL;
    DcMotorEx bR;
    DcMotorEx bL;
    GoBildaPinpointDriver pinpoint;

    public Drive(GamepadEx g, GoBildaPinpointDriver pinpoint,
                 DcMotorEx fR, DcMotorSimple.Direction fRD,
                 DcMotorEx fL, DcMotorSimple.Direction fLD,
                 DcMotorEx bR, DcMotorSimple.Direction bRD,
                 DcMotorEx bL, DcMotorSimple.Direction bLD) {

        this.fR = fR;
        this.fL = fL;
        this.bR = bR;
        this.bL = bL;

        fR.setDirection(fRD);
        fL.setDirection(fLD);
        bR.setDirection(bRD);
        bL.setDirection(bLD);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        movement(g);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void movement(GamepadEx g) {
        double drive = -g.getLeftY();
        double strafe = g.getLeftX();
        double turn = g.getRightX();

        double fRPower = drive - strafe - turn;
        double fLPower = drive + strafe + turn;
        double bRPower = drive + strafe - turn;
        double bLPower = drive - strafe + turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(fRPower),
                Math.max(Math.abs(fLPower),
                Math.max(Math.abs(bRPower), Math.abs(bLPower)))
        ));

        fRPower /= max;
        fLPower /= max;
        bRPower /= max;
        bLPower /= max;

        fR.setPower(fRPower);
        fL.setPower(fLPower);
        bR.setPower(bRPower);
        bL.setPower(bLPower);
    }

    public DcMotorEx getFr() {
        return fR;
    }

    public DcMotorEx getFl() {
        return fL;
    }

    public DcMotorEx getBr() {
        return bR;
    }

    public DcMotorEx getBl() {
        return bL;
    }

    public double getX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    @Override
    public void periodic() {
        pinpoint.update();
    }
}
