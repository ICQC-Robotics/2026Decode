package org.firstinspires.ftc.teamcode.Mode.raihancode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisSubsystem extends SubsystemBase {
    // Four drive motors
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    public ChassisSubsystem(HardwareMap hardwareMap) {

    }

    public void setPower(double left, double right) {
    }
}
