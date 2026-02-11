package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    DcMotorEx intake;
    Servo intakeServo;
    int slowShootTicks = 100;
    int intakeTarget;
    double intakeSpeed;
    double kP = 0.01;

    public Intake(DcMotorEx intake, Servo intakeServo, DcMotorSimple.Direction dir) {
        this.intake = intake;
        this.intakeServo = intakeServo;
        intake.setDirection(dir);
        intakeTarget = -157;
        intakeSpeed = 0;
    }

    public void set(double pos) {
        intakeServo.setPosition(pos);
    }

    public void setSpeed(double speed) {
        intakeSpeed = speed;
        intakeTarget = -157;
    }
    public void shootOne(){
        intakeTarget = intake.getCurrentPosition() + slowShootTicks;
    }
    @Override
    public void periodic(){
        if(intakeTarget != -157){
             intake.setPower(kP * (intakeTarget - intake.getCurrentPosition()));
        }
        else {
            intake.setPower(intakeSpeed);
        }
    }
}
