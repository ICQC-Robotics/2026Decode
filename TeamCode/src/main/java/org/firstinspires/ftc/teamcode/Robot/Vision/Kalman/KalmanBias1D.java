package org.firstinspires.ftc.teamcode.Robot.Vision.Kalman;
public final class KalmanBias1D {
    private double bias;
    private double P;
    private double Qrate;
    private double R;

    public KalmanBias1D(double initialBias, double initialP, double qRate, double r) {
        this.bias = initialBias;
        this.P = initialP;
        this.Qrate = qRate;
        this.R = r;
    }

    public void setQrate(double qRate) { this.Qrate = qRate; }
    public void setR(double r) { this.R = r; }

    public void reset(double initialBias, double initialP) {
        this.bias = initialBias;
        this.P = initialP;
    }

    public void predict(double dtSec) {
        if (dtSec < 0) dtSec = 0;
        P += Qrate * dtSec;
        if (P < 0) P = 0;
    }

    public void update(double z) {
        double K = P / (P + R);
        bias = bias + K * (z - bias);
        P = (1.0 - K) * P;
        if (P < 0) P = 0;
    }

    public double getBias() { return bias; }
    public double getP() { return P; }

    public double getK() {
        return P / (P + R);
    }
}
