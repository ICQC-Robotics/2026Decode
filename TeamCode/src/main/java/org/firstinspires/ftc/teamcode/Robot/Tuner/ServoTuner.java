package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="Servo tuner")
public class ServoTuner extends OpMode {
    Servo servo1, servo2;
    int s1, s2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "cover");
        servo2 = hardwareMap.get(Servo.class, "intakeServo");
    }

    @Override
    public void loop() {
        servo1.setPosition(s1);
        servo2.setPosition(s2);
    }
}
