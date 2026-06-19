package org.firstinspires.ftc.teamcode.Robot.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Splits the camera frame into 3 full-height vertical boxes, one per artifact pickup zone, and
 * reports how much green/purple artifact color is covering each box. Box edges are computed
 * (not hand-picked) from the camera's mount geometry and each zone's pickup-path endpoints, so
 * they line up with where a real artifact in that zone would appear on screen.
 *
 * Reports density purely by on-screen position (left/mid/right) -- it has no notion of alliance.
 * Vision.java maps left/mid/right to zone 1/2/3 based on alliance, since the field (and therefore
 * which physical zone appears on which side of the image) mirrors between red and blue.
 */
public class ArtifactZoneProcessor implements VisionProcessor {

    // ── Camera mount geometry, relative to the shoot pose (BLUE frame; matches ZayansFarAuto) ──
    public static double SHOOT_POS_X = 45.0;
    public static double SHOOT_POS_Y = 9.0;
    public static double CAMERA_BACK_OFFSET_IN = 7.0;  // camera sits this far BEHIND robot center

    // Pickup-path endpoints per zone (near/far corner the robot drives through), index 1..3.
    private static final double[] ZONE_NEAR_X = {0, 8.5, 8.5, 8.5};
    private static final double[] ZONE_FAR_X  = {0, 11.5, 11.5, 11.5};
    private static final double[] ZONE_Y       = {0, 8.0, 20.0, 32.0};

    // Horizontal field of view of the MOUNTED WEBCAM, in degrees. MUST be set to match the actual
    // camera (check its spec sheet / run a calibration) -- this directly determines box placement.
    public static double HORIZONTAL_FOV_DEG = 90.0;

    // HSV thresholds for the two artifact colors (starting point carried over from
    // IntakeColorProcessing.java's tuned values -- re-verify under match lighting).
    private final Scalar GREEN_LOW   = new Scalar(45, 60, 40);
    private final Scalar GREEN_HIGH  = new Scalar(85, 255, 255);
    private final Scalar PURPLE_LOW  = new Scalar(130, 60, 40);
    private final Scalar PURPLE_HIGH = new Scalar(165, 255, 255);

    private final Mat hsv          = new Mat();
    private final Mat maskGreen    = new Mat();
    private final Mat maskPurple   = new Mat();
    private final Mat combinedMask = new Mat();

    private int frameWidth, frameHeight;
    // Pixel boundaries of the 3 on-screen regions, left to right.
    private final int[] regionLeftPx  = new int[3];
    private final int[] regionRightPx = new int[3];

    // Published as a whole new array each frame so readers on another thread never see a
    // half-updated snapshot (safe publication via a volatile reference, no locking needed).
    private volatile double[] regionDensityPct = {0, 0, 0};

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;
        computeRegionBoundaries();
    }

    private void computeRegionBoundaries() {
        double[] centerBearingDeg = new double[4];
        for (int z = 1; z <= 3; z++) {
            double bNear = bearingDeg(ZONE_NEAR_X[z], ZONE_Y[z]);
            double bFar  = bearingDeg(ZONE_FAR_X[z],  ZONE_Y[z]);
            centerBearingDeg[z] = (bNear + bFar) / 2.0;
        }

        // centerBearingDeg[1] > [2] > [3] (zone1 is slightly camera-right, zone3 far camera-left).
        // Split the frame at the angular midpoint between neighboring zones; outer zones run to
        // the frame edges so the 3 boxes always tile the whole width with no gaps/overlap.
        double boundary12 = (centerBearingDeg[1] + centerBearingDeg[2]) / 2.0;
        double boundary23 = (centerBearingDeg[2] + centerBearingDeg[3]) / 2.0;

        int pxBoundary12 = bearingToPixel(boundary12);
        int pxBoundary23 = bearingToPixel(boundary23);

        regionLeftPx[0]  = 0;             regionRightPx[0] = pxBoundary23;   // leftmost  = zone3 side
        regionLeftPx[1]  = pxBoundary23;  regionRightPx[1] = pxBoundary12;   // middle    = zone2 side
        regionLeftPx[2]  = pxBoundary12;  regionRightPx[2] = frameWidth;     // rightmost = zone1 side
    }

    private static double cameraX() { return SHOOT_POS_X - CAMERA_BACK_OFFSET_IN; }
    private static double cameraY() { return SHOOT_POS_Y; }

    /** Bearing (deg) from the camera to a field point; camera faces -x, camera-right = -y. */
    private static double bearingDeg(double targetX, double targetY) {
        double forward = cameraX() - targetX;
        double right   = cameraY() - targetY;
        return Math.toDegrees(Math.atan2(right, forward));
    }

    private int bearingToPixel(double bearingDegVal) {
        double halfFovRad = Math.toRadians(HORIZONTAL_FOV_DEG / 2.0);
        double norm = Math.tan(Math.toRadians(bearingDegVal)) / Math.tan(halfFovRad);  // -1..+1 across the FOV
        int px = (int) Math.round(frameWidth / 2.0 + norm * frameWidth / 2.0);
        return clamp(px, 0, frameWidth);
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, maskGreen);
        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, maskPurple);
        Core.bitwise_or(maskGreen, maskPurple, combinedMask);

        double[] pct = new double[3];
        for (int i = 0; i < 3; i++) {
            int left = regionLeftPx[i], right = regionRightPx[i];
            if (right <= left) continue;

            Mat region = combinedMask.submat(0, frameHeight, left, right);
            int litPixels = Core.countNonZero(region);
            region.release();

            double area = (double) (right - left) * frameHeight;
            pct[i] = 100.0 * litPixels / area;   // % of this box covered by artifact color
        }
        regionDensityPct = pct;

        return null;
    }

    /** % of each on-screen box (left/mid/right) covered by artifact color. Not alliance-aware. */
    public double[] getRegionDensityPct() {
        return regionDensityPct;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                             float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4 * scaleCanvasDensity);
        paint.setColor(Color.YELLOW);

        double[] pct = regionDensityPct;
        for (int i = 0; i < 3; i++) {
            float left  = regionLeftPx[i]  * scaleBmpPxToCanvasPx;
            float right = regionRightPx[i] * scaleBmpPxToCanvasPx;
            canvas.drawRect(left, 0, right, onscreenHeight, paint);
        }
    }
}
