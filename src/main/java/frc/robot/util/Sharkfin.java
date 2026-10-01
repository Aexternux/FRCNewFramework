// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class Sharkfin {


    private double intensity;
    private double deviation;

    /**
     * do doo do do do doo
     * @param intensity Intensity of approach
     * @param deviation width of approach
     */
    public Sharkfin(double intensity, double deviation){
        setGains(intensity, deviation);
    }

    /**
     * do doo do do do doo
     * @param intensity Intensity of approach
     * @param deviation width of approach
     */
    public void setGains(double intensity, double deviation){
        this.intensity = intensity;
        this.deviation = deviation;
    }

    // Calculates desired output z based on an orthogonal set of errors z and x
    public double calculate(double approach_error, double balance_error) {
        return intensity * Util.fast_sigmoid(approach_error) * Util.gaussian(balance_error, deviation);
    }

}
