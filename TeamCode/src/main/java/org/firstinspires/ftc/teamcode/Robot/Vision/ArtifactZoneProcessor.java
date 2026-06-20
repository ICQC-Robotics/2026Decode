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
 * Splits the camera frame into 3 evenly-sized boxes, one per artifact pickup zone, and reports
 * how much green/purple artifact color is covering each. Boxes tile the full frame edge-to-edge
 * (no dead space) and, for RED alliance, zone1 is leftmost. For BLUE, the whole layout is
 * mirrored horizontally (the field -- and therefore which physical zone the camera sees on which
 * side -- mirrors between alliances), so call {@link #setAlliance(boolean)} whenever the
 * alliance is known/changes.
 */
public class ArtifactZoneProcessor implements VisionProcessor {

    // Layout, as fractions of frame width/height, for RED alliance. Mirrored for BLUE.
    // Boxes tile the FULL frame edge-to-edge in both width and height -- no dead space.
    private static final double BOX_START_FRAC  = 0.0;
    private static final double BOX_WIDTH_FRAC  = 1.0 / 3.0;
    private static final double BOX_HEIGHT_FRAC = 1.0;

    // Left-edge fraction of each zone's box in the RED layout (index 1..3); zone1 is leftmost
    // (camera is mounted flipped, so the left/right sense is reversed from a plain mirror).
    private static final double[] ZONE_START_FRAC_RED = {
            0,
            BOX_START_FRAC,                          // zone1: leftmost box
            BOX_START_FRAC + BOX_WIDTH_FRAC,         // zone2: middle box
            BOX_START_FRAC + 2 * BOX_WIDTH_FRAC      // zone3: rightmost box
    };

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
    private int boxTopPx, boxBottomPx;

    private volatile boolean blueAlliance = true;

    // Published as a whole new array each frame so readers on another thread never see a
    // half-updated snapshot (safe publication via a volatile reference, no locking needed).
    private volatile double[] zoneDensityPct = {0, 0, 0, 0};  // index 1..3

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;
        boxTopPx    = 0;
        boxBottomPx = (int) Math.round(frameHeight * BOX_HEIGHT_FRAC);
    }

    /** Call whenever the current alliance is known/changes -- flips the box layout to match. */
    public void setAlliance(boolean isBlue) {
        blueAlliance = isBlue;
    }

    private int zoneLeftPx(int z) {
        return (int) Math.round(zoneStartFrac(z) * frameWidth);
    }

    private int zoneRightPx(int z) {
        return (int) Math.round((zoneStartFrac(z) + BOX_WIDTH_FRAC) * frameWidth);
    }

    private double zoneStartFrac(int z) {
        double startFrac = ZONE_START_FRAC_RED[z];
        if (!blueAlliance) return startFrac;
        // BLUE: mirror the box horizontally -- [start, start+width] -> [1-(start+width), 1-start].
        return 1.0 - startFrac - BOX_WIDTH_FRAC;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, maskGreen);
        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, maskPurple);
        Core.bitwise_or(maskGreen, maskPurple, combinedMask);

        double[] pct = new double[4];
        double area = (frameWidth * BOX_WIDTH_FRAC) * (frameHeight * BOX_HEIGHT_FRAC);
        for (int z = 1; z <= 3; z++) {
            int left = zoneLeftPx(z), right = zoneRightPx(z);
            if (right <= left) continue;

            Mat box = combinedMask.submat(boxTopPx, boxBottomPx, left, right);
            int litPixels = Core.countNonZero(box);
            box.release();

            pct[z] = 100.0 * litPixels / area;  // % of this box covered by artifact color
        }
        zoneDensityPct = pct;

        return null;
    }

    /** % of each zone's box covered by artifact color, index 1..3. Already alliance-correct. */
    public double[] getZoneDensityPct() {
        return zoneDensityPct;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                             float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4 * scaleCanvasDensity);
        paint.setColor(Color.YELLOW);
        paint.setTextSize(32 * scaleCanvasDensity);

        float top    = boxTopPx    * scaleBmpPxToCanvasPx;
        float bottom = boxBottomPx * scaleBmpPxToCanvasPx;

        for (int z = 1; z <= 3; z++) {
            float left  = zoneLeftPx(z)  * scaleBmpPxToCanvasPx;
            float right = zoneRightPx(z) * scaleBmpPxToCanvasPx;
            canvas.drawRect(left, top, right, bottom, paint);
            canvas.drawText("z" + z, left + 8, top + 32 * scaleCanvasDensity, paint);
        }
    }
}
