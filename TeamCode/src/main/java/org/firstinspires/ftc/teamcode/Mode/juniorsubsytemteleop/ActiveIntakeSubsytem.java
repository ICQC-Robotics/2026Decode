package org.firstinspires.ftc.teamcode.Mode.juniorsubsytemteleop;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActiveIntakeSubsytem extends SubsystemBase {

    private final DcMotorEx intake;

    public ActiveIntakeSubsytem(HardwareMap hMap) {
        intake = hMap.get(DcMotorEx.class, "intake_motor");

    }

    public void runIn() {
        intake.setPower(1.0);
    }

    public void runOut() {
        intake.setPower(-1.0);
    }

    public void stop() {
        intake.setPower(0.0);
    }


    @Override
    public void periodic() {

    }
}