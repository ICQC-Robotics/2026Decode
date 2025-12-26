package org.firstinspires.ftc.teamcode;
//Shows So many errors because i do not know how to properly import these 3 since I used youtube video for a little help

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "FarAuto", group = "Auto")
public class FarAuto extends LinearOpMode {

    DcMotor fl, fr, bl, br;

    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        driveForward(1500, 0.5);
        strafeLeft(1500, 0.5);
        driveForward(800, 0.5);
        stopDrive();
    }

    void driveForward(int ticks, double power) {
        setTarget(ticks, ticks, ticks, ticks);
        runToPos(power);
    }

    void driveBackward(int ticks, double power) {
        setTarget(-ticks, -ticks, -ticks, -ticks);
        runToPos(power);
    }

    void strafeLeft(int ticks, double power) {
        setTarget(-ticks, ticks, ticks, -ticks);
        runToPos(power);
    }

    void strafeRight(int ticks, double power) {
        setTarget(ticks, -ticks, -ticks, ticks);
        runToPos(power);
    }

    void setTarget(int flT, int frT, int blT, int brT) {
        fl.setTargetPosition(flT);
        fr.setTargetPosition(frT);
        bl.setTargetPosition(blT);
        br.setTargetPosition(brT);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void runToPos(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        while (opModeIsActive() && fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {}
    }

    void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
