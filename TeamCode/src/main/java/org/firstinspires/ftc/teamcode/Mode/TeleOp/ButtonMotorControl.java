package org.firstinspires.ftc.teamcode.Mode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ButtonMotorControl", group = "TeleOp")
public class ButtonMotorControl extends LinearOpMode {

    private DcMotor m1;

    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m1.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                m1.setPower(1.0);
            } else {
                m1.setPower(0);
            }

            telemetry.addData("Motor Power", m1.getPower());
            telemetry.update();
        }
    }
}
