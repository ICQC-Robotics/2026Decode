package org.firstinspires.ftc.teamcode.Robot.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class IntakeColorProcessing implements VisionProcessor {

    public enum Zone {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    private final Mat hsv = new Mat();

    private final Mat maskPurple = new Mat();
    private final Mat maskGreen = new Mat();

    private final Mat combinedMask = new Mat();

    private volatile Zone detectedZone = Zone.NONE;

    private final Scalar PURPLE_LOW  = new Scalar(130, 60, 40);
    private final Scalar PURPLE_HIGH = new Scalar(165, 255, 255);

    private final Scalar GREEN_LOW  = new Scalar(45, 60, 40);
    private final Scalar GREEN_HIGH = new Scalar(85, 255, 255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, maskPurple);
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, maskGreen);

        Core.bitwise_or(maskPurple, maskGreen, combinedMask);

        int width = combinedMask.cols();
        int height = combinedMask.rows();
        int third = width / 3;

        Mat leftMat   = combinedMask.submat(0, height, 0, third);
        Mat centerMat = combinedMask.submat(0, height, third, 2 * third);
        Mat rightMat  = combinedMask.submat(0, height, 2 * third, width);

        double leftValue   = Core.sumElems(leftMat).val[0];
        double centerValue = Core.sumElems(centerMat).val[0];
        double rightValue  = Core.sumElems(rightMat).val[0];

        leftMat.release();
        centerMat.release();
        rightMat.release();

        double threshold = 1000000;

        double max = Math.max(leftValue, Math.max(centerValue, rightValue));

        if (max < threshold) {
            detectedZone = Zone.NONE;
        } else if (max == leftValue) {
            detectedZone = Zone.RIGHT;
        } else if (max == centerValue) {
            detectedZone = Zone.CENTER;
        } else {
            detectedZone = Zone.LEFT;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Zone getDetectedZone() {
        return detectedZone;
    }
}
