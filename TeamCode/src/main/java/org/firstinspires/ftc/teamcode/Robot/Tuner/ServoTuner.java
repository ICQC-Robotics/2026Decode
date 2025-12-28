package org.firstinspires.ftc.teamcode.Robot.Tuner;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="Servo tuner")
public class ServoTuner extends OpMode {
    Servo servo0, servo1, servo2, servo3;
    public static double s0, s1, s2, s3;

    @Override
    public void init() {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
    }

    @Override
    public void loop() {
        servo0.setPosition(s0);
        servo1.setPosition(s1);
        servo2.setPosition(s2);
        servo3.setPosition(s3);
    }
}
