// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class SpikeFilter {

    private double Mabsolute; // max absolute difference between 2 values
    private double Mratio; // max absolute ratio of difference between 2 values / previous value
    private int n; // 0 is off; suggest 1 or maybe rarely 2
    private int c;
    private double prevInput;
    private boolean firstTime = true;

    /**
     * 
     * @param Mabsolute difference to suppress
     * @param Mratio    difference to suppress
     * @param n         count periods, 0 is off; suggest 1 or maybe rarely 2
     */
    public SpikeFilter(double Mabsolute, double Mratio, int n) {
        this.Mabsolute = Mabsolute;
        this.Mratio = Mratio;
        this.n = n;
        this.c = 0;
    }

    public double calculate(double rawInput) {
        if (firstTime) {
            firstTime = false;
            prevInput = rawInput;
        }

        if ((Math.abs(rawInput - prevInput) > Mabsolute ||
                Math.abs((rawInput - prevInput)) > Math.abs(Mratio * prevInput))
                &&
                c < n) { // large change; use previous good value
            c++; // count number of times large change
            return prevInput;
        } else { // normal input or large change is holding
            c = 0;
            prevInput = rawInput;
            return rawInput;
        }
    }

    public void setLimit(double Mabsolute, double Mratio) {
        this.Mabsolute = Mabsolute;
        this.Mratio = Mratio;
        this.c = 0;
    }
}
