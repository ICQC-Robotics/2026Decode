package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    DcMotorEx intake1, intake2;

    public Intake(DcMotorEx intake1, DcMotorEx intake2, DcMotorSimple.Direction dir1, DcMotorSimple.Direction dir2) {
        this.intake1 = intake1;
        this.intake2 = intake2;
        intake1.setDirection(dir1);
        intake2.setDirection(dir2);
    }

    public void setSpeed(double speed) {
        intake1.setPower(speed);
        intake2.setPower(speed);
    }
}
