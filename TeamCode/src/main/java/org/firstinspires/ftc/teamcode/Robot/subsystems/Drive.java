package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

public class Drive extends SubsystemBase {
    public final MecanumDrive mD;
    private Pose2d p = new Pose2d(0, 0, 0);
    private final DcMotorEx fR, fL, bR, bL;
    private GamepadEx g1;

    public Drive(HardwareMap h, GamepadEx g1,
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

        fR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        mD = new MecanumDrive(h, p);
        this.g1 = g1;
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

    @Override
    public void periodic() {
        mD.localizer.update();
        movement(g1);
    }
}
