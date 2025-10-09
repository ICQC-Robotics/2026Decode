package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    DcMotorEx intake;
    Servo intakeServo;

    public Intake(DcMotorEx intake, Servo intakeServo) {
        this.intake = intake;
        this.intakeServo = intakeServo;
    }

    public void set(double pos) {
        intakeServo.setPosition(pos);
    }

    public void setSpeed(int speed) {
        intake.setPower(speed);
    }
}
