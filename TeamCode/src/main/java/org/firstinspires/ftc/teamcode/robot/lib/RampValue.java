package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/***
 * Ramp the change between two values. This is called in a loop to perform regular calculation.
 */
public class RampValue {

    private int s_upDilation = 1;                   // dilation of the quadratic that increases
    private int s_downDilation = 1;                 // dilation of the quadratic that decreases
    private double prevValue = 0.0;                 // previous value for calculating change
    private ElapsedTime time = new ElapsedTime();   // timer for timing functions
    private double maxRampAccel = 1.0;              // clamping accel for trapezoid
    private double origValue = 0.0;                 // starting value
    private RampMode rampMode;                      // the chosen ramp mode
    private double setPoint = 0.0;                  // the desired ending value
    private double prevTime = 0.0;                  // the time of the previous calculation
    private Telemetry telemetry;

    /***
     * Constructor for no preset values
     */
    public RampValue(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /***
     * Constructor with preset values
     * @param mode the ramp mode to be set [SCURVE or TRAPEZOID]
     * @param maxRampAccel the maximum allowed acceleration
     * @param origValue the starting value
     * @param setPoint the desired final value
     * @param dilations the two dilations for up and down quadratics [optional]
     */
    public RampValue(RampMode mode, double maxRampAccel, double origValue, double setPoint,
                     int...dilations){
        this.rampMode = mode;
        this.maxRampAccel = Math.abs(maxRampAccel);
        this.origValue = Math.abs(origValue);
        this.setPoint = Math.abs(setPoint);

        if(dilations != null && dilations.length > 1){
            s_upDilation = 1 / dilations[1];
            s_downDilation = 1 / dilations[2];
        }
        else{
            s_upDilation = 1;
            s_downDilation = 1;
        }
    }

    /***
     * Set the ramp mode
     * @param mode the ramp mode to set [SCURVE or TRAPEZOID]
     */
    public void setMode(RampMode mode){
        this.rampMode = mode;
    }

    /***
     * Set the maximum acceleration
     * @param accel the maximum allowed acceleration
     */
    public void setMaxRampAccel(double accel){
        this.maxRampAccel = Math.abs(accel);
    }

    /***
     * Set the dilation of the increasing part of the SCURVE.
     * Will be 1 / dilation meaning that larger numbers will give slower curves.
     * @param scurveUpDilation the dilation to set
     */
    public void setScurveUpDilation(int scurveUpDilation){
        this.s_upDilation = 1 / scurveUpDilation;
    }

    /***
     * Set the dilation of the decreasing part of the SCURVE.
     * Will be 1 / dilation meaning that larger numbers will give slower curves.
     * @param scurveDownDilation the dilation to set
     */
    public void setScurveDownDilation(int scurveDownDilation){
        this.s_downDilation = 1 / scurveDownDilation;
    }

    /***
     * Set the original input or starting value
     * @param origValue the original or starting value
     */
    public void setOrigInput(double origValue){
        this.origValue = Math.abs(origValue);
    }

    /***
     * Set the desired value or setpoint
     * @param setPoint the setpoint
     */
    public void setSetPoint(double setPoint){
        this.setPoint = Math.abs(setPoint);
    }

    /***
     * Starts timing for the calculations.
     * This needs to be run immediately before starting the calculations.
     */
    public void startTime(){
        time.reset();
        prevTime = 0.0;
    }

    /***
     * Ramp up using the clamp method and the maximum.
     * MUST be run in a loop.
     * @return the expected value for the ramp at this time
     */
    public double ramp(){
        return clamp(setPoint);
    }

    /***
     * Ramp up using the clamp method and the maximum. Limit acceleration to @maxRampAccel.
     * MUST be run in a loop.
     * @param maxRampAccel the maximum allowed acceleration
     * @return the expected value for the ramp at this time
     */
    public double ramp(double maxRampAccel){
        setMaxRampAccel(maxRampAccel);
        return ramp();
    }

    /***
     * Ramp up using the clamp method and the @setPoint. Limit acceleration to @maxRampAccel.
     * MUST be run in a loop.
     * @param setPoint the desired end value
     * @param maxRampAccel the maximum allowed acceleration
     * @return the expected value for the ramp at this time
     */
    public double ramp(double setPoint, double maxRampAccel){
        setSetPoint(setPoint);
        setMaxRampAccel(maxRampAccel);
        return ramp();
    }

    /***
     * Clamps the current value to the expected value given the parameters.
     * MUST be run in a loop. Does not handle -ve to +ve or vice versa.
     * @param currentValue the current value to clamp down
     * @return returns the currentValue if below the expectedValue, otherwise the expectedValue
     */
    public double clamp(double currentValue){
        int sign = 1;

        // set up time values for calculation
        double currentTime = time.time();
        double timeDelta = currentTime - prevTime;

        // set up sign value
        if(currentValue < 0){
            sign = -1;
        }

        // all calculations to be done in +ve space
        currentValue = Math.abs(currentValue);

        // choose how to set the current value based on ramp mode
        switch (rampMode) {
            case TRAPEZOID: // increase at a steady rate, infinite jerk on entry and exit
                if ((currentValue - prevValue) / timeDelta > maxRampAccel) {
                    currentValue = (prevValue + (maxRampAccel * timeDelta));
                }

                break;
            case SCURVE: // smooth transition, controlled jerk
                double expectedValue;

                // the gradient at which to swap between increasing velocity and decreasing velocity
                double swapGradient = 2 * Math.sqrt((Math.abs(setPoint - origValue))
                        * (s_upDilation * s_downDilation) / (s_upDilation + s_downDilation));

                // the amount of time it will take for the down curve to reach the desired value
                double remainingTime =
                        Math.sqrt((setPoint - currentValue) / s_downDilation);

                // current gradient of the changing acceleration  section
                double upGradient = 2 * s_upDilation * currentTime;
                double downGradient = 2 * s_downDilation * remainingTime;

                if(swapGradient >= maxRampAccel){
                    // we have constant gradient segment in the middle
                    if(upGradient < maxRampAccel){ // increasing acceleration
                        expectedValue = s_upDilation * Math.pow(currentTime, 2);
                    }
                    else if(downGradient <= maxRampAccel){ // decreasing acceleration
                        expectedValue = setPoint
                                - s_downDilation * Math.pow((remainingTime), 2);
                    }
                    else { //steady
                        expectedValue = (prevValue + (maxRampAccel * timeDelta));
                    }
                }
                else{ // our two curves meet without constant gradient
                    if(upGradient < swapGradient){ // before swapping between curves
                        expectedValue = s_upDilation * Math.pow(currentTime, 2);
                    }
                    else{ // after swapping to the decreasing curve
                        expectedValue = setPoint
                                - s_downDilation * Math.pow((remainingTime), 2);
                    }
                }

                if(currentValue > expectedValue){ // clamp the output
                    currentValue = expectedValue;
                }
        }

        prevValue = currentValue;
        prevTime = currentTime;

        return sign * currentValue;
    }
}
