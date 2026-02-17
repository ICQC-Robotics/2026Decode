package org.firstinspires.ftc.teamcode.Robot.subsystems;

public final class KalmanFiltering {
    private double x;
    private double p;
    private double Q;
    private double R;
    private boolean initialized = false;

    public KalmanFiltering(double Q, double R, double p0) {
        this.Q = Q;
        this.R = R;
        this.p = p0;
    }

    public void setQ(double Q) {
        this.Q = Q;
    }
    public void setR(double R) {
        this.R = R;
    }

    public void reset(double x0, double p0) {
        this.x = x0;
        this.p = p0;
        this.initialized = true;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void predict(double u) {
        if (!initialized) {
            return;
        }

        x += u;
        p += Q;
    }

    //K = p/(p+R), x = x + K(z-x), p = (1-K)p
    public void update(double z) {
        if (!initialized) {
            reset(z, p);
            return;
        }
        double K = p / (p + R);
        x = x + K * (z - x);
        p = (1.0 - K) * p;
    }

    public double getX() {
        return x;
    }
    public double getP() {
        return p;
    }
}

