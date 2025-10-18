package org.firstinspires.ftc.teamcode.Mode.juniorsubsytemteleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ShooterTransferIntakeControl", group = "TeleOp")
public class TransferShooterIntake extends LinearOpMode {

    private DcMotor shooter;
    private DcMotor transfer;
    private DcMotor intake;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotor.class, "m3");
        transfer = hardwareMap.get(DcMotor.class, "m2");
        intake = hardwareMap.get(DcMotor.class, "m1");

        shooter.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooter.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double shooterPower = 0;
            double transferPower = 0;
            double intakePower = 0;

            if (gamepad1.right_bumper) {
                shooterPower = -1.0;
            } else if (gamepad1.left_bumper) {
                shooterPower = 1.0;
            }

            if (gamepad1.right_trigger > 0.1) {
                transferPower = -1.0;
            } else if (gamepad1.left_trigger > 0.1) {
                transferPower = 1.0;
            }

            if (gamepad1.x) {
                intakePower = 1.0;
            } else if (gamepad1.y) {
                intakePower = -1.0;
            }

            if (gamepad1.a) {
                shooterPower = -1.0;
                transferPower = -1.0;
                intakePower = 1.0;
            } else if (gamepad1.b) {
                shooterPower = 1.0;
                transferPower = 1.0;
                intakePower = -1.0;
            }

            // Removed D-Pad Up control

            shooter.setPower(shooterPower);
            transfer.setPower(transferPower);
            intake.setPower(intakePower);

            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Transfer Power", transferPower);
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }
}
