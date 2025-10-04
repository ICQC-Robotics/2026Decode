package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="PracticeAutoSimple", group="Practice")
public class practiceauto extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialize motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // reverse left motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // drive forward ~30 inches (adjust sleep to match your robot)
        setPower(0.5, 0.5);
        sleep(1000);
        setPower(0,0);

        // turn right 90 degrees
        setPower(0.5, -0.5);
        sleep(500);
        setPower(0,0);

        // drive forward ~15 inches
        setPower(0.5, 0.5);
        sleep(500);
        setPower(0,0);
    }

    private void setPower(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }
}
