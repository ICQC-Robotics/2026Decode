package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive extends SubsystemBase {
    DcMotorEx fR;
    DcMotorEx fL;
    DcMotorEx bR;
    DcMotorEx bL;

    public Drive(GamepadEx g, DcMotorEx fR, DcMotorSimple.Direction fRD,
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
}
