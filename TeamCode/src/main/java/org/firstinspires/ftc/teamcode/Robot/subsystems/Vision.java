package org.firstinspires.ftc.teamcode.Robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot.Vision.IntakeColorProcessing;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision extends SubsystemBase {

    //ll
    private final Limelight3A limelight;
    private LLResult lastResult;
    public final double turretRadius = 2.75;
    public final double turretOffsetX = 0;
    public final double turretOffsetY = 3;

    //webcam
    private VisionPortal portal;
    private IntakeColorProcessing processor;

    private static final double CAMERA_HEIGHT = 8.25;
    private static final double CAMERA_PITCH_DEG = 10.0;

    private static final double HFOV_DEG = 82.0;
    private static final double VFOV_DEG = 56.0;

    // inches from robot center
    private static final double CAMERA_FORWARD_OFFSET = 4.0;
    private static final double CAMERA_LEFT_OFFSET = 0.0;

    public Vision(Limelight3A vision, HardwareMap hardwareMap, String cameraName) {
        limelight = vision;
        limelight.pipelineSwitch(0);
        limelight.start();

        processor = new IntakeColorProcessing();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .addProcessor(processor)
                .build();
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
    }

    public Pose getBotposeMT1(double t) {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return null;

        t -= 135;
        double tRad = Math.toRadians(t);

        Pose3D pose = r.getBotpose();
        if (pose == null) return null;

        // p is the camera pose in field coordinates (Limelight without a robot-body
        // transform configured, so it reports the camera center as the "bot" position).
        Pose p = new Pose(
                pose.getPosition().x * 39.37007874,
                pose.getPosition().y * 39.37007874,
                pose.getOrientation().getYaw(AngleUnit.RADIANS)
        );

        // The turret rotates CW (positive t) relative to robot forward, which decreases
        // the camera's field heading. So: camera_heading = robot_heading - tRad.
        // Recover robot heading before modifying p, since withX/withY need camHeadRad.
        double camHeadRad = p.getHeading();
        p.setHeading(camHeadRad + tRad);  // robot heading

        // Move from camera center to turret center: camera is turretRadius inches ahead
        // of the turret pivot along the camera's boresight direction in field coordinates.
        p = p.withX(p.getX() - turretRadius * Math.cos(camHeadRad));
        p = p.withY(p.getY() - turretRadius * Math.sin(camHeadRad));

        // Move from turret center to robot center: turret pivot is turretOffsetY inches
        // ahead of robot center along the robot's forward direction in field coordinates.
        double robotHeadRad = p.getHeading();
        p = p.withX(p.getX() - turretOffsetY * Math.cos(robotHeadRad));
        p = p.withY(p.getY() - turretOffsetY * Math.sin(robotHeadRad));

        return p;
    }

    public void updateRobotYawDeg(double robotYawDeg) {
        limelight.updateRobotOrientation(robotYawDeg);
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

    public static class ArtifactTarget {
        public final double x;
        public final double y;
        public final double density;

        public ArtifactTarget(double x, double y, double density) {
            this.x = x;
            this.y = y;
            this.density = density;
        }
    }

    public ArtifactTarget getArtifactTarget() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        double[] python = result.getPythonOutput();

        if (python == null || python.length < 3) {
            return null;
        }

        return new ArtifactTarget(
                python[0], // normalized x
                python[1], // normalized y
                python[2]  // density
        );
    }

    public double getArtifactOffsetX() {
        LLResult result = limelight.getLatestResult();

        if(result == null || !result.isValid())
            return 0;

        return result.getPythonOutput()[0];
    }

    public double getArtifactOffsetY() {
        LLResult result = limelight.getLatestResult();

        if(result == null || !result.isValid())
            return 0;

        return result.getPythonOutput()[1];
    }

    public double getArtifactDensity() {
        LLResult result = limelight.getLatestResult();

        if(result == null || !result.isValid())
            return 0;

        return result.getPythonOutput()[2];
    }

    public Pose getArtifactFieldPose(Pose robotPose) {

        ArtifactTarget target = getArtifactTarget();

        if(target == null)
            return null;

        if(target.density < 0.003)
            return null;

        double normX = target.x;
        double normY = target.y;

        double yawDeg =
                normX * (HFOV_DEG / 2.0);

        double pitchDeg =
                CAMERA_PITCH_DEG +
                        normY * (VFOV_DEG / 2.0);

        if(pitchDeg <= 1)
            return null;

        double forwardDistance =
                CAMERA_HEIGHT /
                        Math.tan(Math.toRadians(pitchDeg));

        double lateralDistance =
                forwardDistance *
                        Math.tan(Math.toRadians(yawDeg));

        double robotHeading =
                robotPose.getHeading();

        double robotForward =
                forwardDistance + CAMERA_FORWARD_OFFSET;

        double robotLeft =
                lateralDistance + CAMERA_LEFT_OFFSET;

        double fieldX =
                robotPose.getX()
                        + robotForward * Math.cos(robotHeading)
                        - robotLeft * Math.sin(robotHeading);

        double fieldY =
                robotPose.getY()
                        + robotForward * Math.sin(robotHeading)
                        + robotLeft * Math.cos(robotHeading);

        return new Pose(
                fieldX,
                fieldY,
                robotHeading
        );
    }
}