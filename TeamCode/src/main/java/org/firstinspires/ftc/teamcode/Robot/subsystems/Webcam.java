package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Vision.IntakeColorProcessing;
import org.firstinspires.ftc.vision.VisionPortal;

public class Webcam extends SubsystemBase {

    private VisionPortal portal;
    private IntakeColorProcessing processor;

    public Webcam(HardwareMap hardwareMap, String cameraName) {

        processor = new IntakeColorProcessing();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(processor)
                .build();
    }

    public IntakeColorProcessing.Zone getZone() {
        return processor.getDetectedZone();
    }

    public void stop() {
        portal.stopStreaming();
    }

    public void close() {
        portal.close();
    }
}
