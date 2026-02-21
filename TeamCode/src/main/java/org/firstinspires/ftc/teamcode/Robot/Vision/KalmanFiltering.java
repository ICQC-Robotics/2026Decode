package org.firstinspires.ftc.teamcode.Robot.Vision;

public final class KalmanFiltering {
    private double x;
    private double P;

    private double Q;
    private double R;

    public KalmanFiltering(
            double initialHeadingDeg,
            double initialP,
            double Q,
            double R
    ) {
        this.x = wrap(initialHeadingDeg);
        this.P = Math.max(0.0, initialP);
        this.Q = Q;
        this.R = R;
    }

    public void update(double imuHeadingDeg,
                       double lastImuHeadingDeg,
                       double limelightHeadingDeg) {

        double u = wrap(imuHeadingDeg - lastImuHeadingDeg);

        x = wrap(x + u);
        P = P + Q;
        if (P < 0) P = 0;

        if (!Double.isFinite(limelightHeadingDeg))
            return;

        double K = P / (P + R);

        double error = wrap(limelightHeadingDeg - x);

        x = wrap(x + K * error);
        P = (1.0 - K) * P;
        if (P < 0) P = 0;
    }

    public double getHeadingDeg() {
        return x;
    }

    private static double wrap(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}