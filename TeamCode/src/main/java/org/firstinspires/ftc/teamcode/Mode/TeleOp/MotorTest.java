package org.firstinspires.ftc.teamcode.Mode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "intake test", group = "TeleOp")
public class MotorTest extends LinearOpMode{
    
    private DcMotor intake;

    public void runOpMode() throws InterruptedException {
         intake = hardwareMap.get(DcMotor.class, "intake");
         
         waitForStart();
         while (opModeIsActive()){
             intake.setPower(1);
         }
    }
}
