package org.firstinspires.ftc.teamcode.Mode.ShariqCode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisSubsystem extends SubsystemBase {
    private final DcMotor leftFront, rightFront, leftRear, rightRear;
 // so basically u initializin n stuff right here from hardware map
    public ChassisSubsystem(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDrivePowers(double lf, double rf, double lr, double rr) {
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }
// stop yo motors here
    public void stop() {
        setDrivePowers(0, 0, 0, 0);
    }
}
