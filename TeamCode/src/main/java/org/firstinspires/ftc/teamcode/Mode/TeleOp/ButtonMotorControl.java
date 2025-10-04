package org.firstinspires.ftc.teamcode.Mode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ButtonMotorControl", group = "TeleOp")
public class ButtonMotorControl extends LinearOpMode {

    private DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotor.class, "m1");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.left_bumper){
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0);
            }

            telemetry.addData("Intake Motor Power", intakeMotor.getPower());
            telemetry.update();
        }
    }
}
