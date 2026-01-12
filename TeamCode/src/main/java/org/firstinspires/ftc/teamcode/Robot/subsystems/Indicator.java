package org.firstinspires.ftc.teamcode.Robot.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Indicator extends SubsystemBase {

    public enum State {
        IDLE(0.0),          // no color
        OUT(0.388),         // yellow
        AIMING(0.333),      // orange
        READY(0.5),         // green
        FIRING(0.722);      // purple
        public final double pos;
        State(double pos) { this.pos = pos; }
    }

    private final ServoImplEx lightOne;
    private final ServoImplEx lightTwo;

    private State state = State.IDLE;

    public Indicator(ServoImplEx lightOne, ServoImplEx lightTwo ) {
        this.lightOne = lightOne;
        this.lightTwo = lightTwo;

        setState(State.IDLE);
    }

    public void setState(State s) {
        state = s;
        lightOne.setPosition(s.pos);
        lightTwo.setPosition(s.pos);

    }

    public State getState() {
        return state;
    }
}
