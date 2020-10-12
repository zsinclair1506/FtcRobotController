package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

/***
 * Ramp the change between two values
 */
public class RampValue {

    private int s_upDilation = 1;
    private int s_downDilation = 1;
    private double prevValue = 0.0;
    private ElapsedTime time = new ElapsedTime();
    private double timeDelta = 0.0;
    private double maxRampAccel = 1.0;
    private double origValue = 0.0;
    private RampMode rampMode;
    private double setPoint = 0.0;
    private double currentTime = 0.0;
    private double prevTime = 0.0;


    public RampValue(){

    }

    public RampValue(RampMode mode, double maxRampAccel, double origValue, double setPoint,
                     int...dilations){
        this.rampMode = mode;
        this.maxRampAccel = maxRampAccel;
        this.origValue = origValue;
        this.setPoint = setPoint;

        if(dilations != null && dilations.length > 1){
            s_upDilation = dilations[1];
            s_downDilation = dilations[2];
        }
        else{
            s_upDilation = 1;
            s_downDilation = 1;
        }
    }

    public void setMode(RampMode mode){
        this.rampMode = mode;
    }

    public void setMaxRampAccel(double accel){
        this.maxRampAccel = accel;
    }

    public void setSUpPower(int power){
        this.s_upDilation = power;
    }

    public void setSDownPower(int power){
        this.s_downDilation = power;
    }

    public void setOrigInput(double origInput){
        this.origValue = origInput;
    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public void startTime(){
        time.reset();
        prevTime = 0.0;
    }

    public double ramp(){

        return 0;
    }

    public double ramp(double maxRampAccel){
        setMaxRampAccel(maxRampAccel);
        return ramp();
    }

    public double clamp(double currentValue){
        int sign = 1;

        currentTime = time.time();
        timeDelta = currentTime - prevTime;

        if(currentValue < 0){
            sign = -1;
        }

        switch (rampMode) {
            case TRAPEZOID:
                if (Math.abs(currentValue - prevValue) / timeDelta > maxRampAccel) {
                    currentValue = (prevValue + (maxRampAccel * timeDelta));
                }

                break;
            case SCURVE:
                double expectedValue = 0.0;
                double upGradient = 2 * s_upDilation * currentTime;

                //the value at which we need to swap to downAccel
                double switchDownValue = (Math.pow(maxRampAccel, 2) / (4 * s_downDilation));
                double switchUpValue = (Math.pow(maxRampAccel, 2) / (4 * s_upDilation));
                if(switchUpValue + switchDownValue > setPoint){
                    //switchover before maxRampAccel

                }

                if(upGradient < maxRampAccel){
                    expectedValue = s_upDilation * Math.pow(currentTime, 2);
                }
                else{
                    expectedValue = (prevValue + (maxRampAccel * timeDelta));
                }

                if(Math.abs(currentValue) > expectedValue){
                    currentValue = expectedValue;
                }

        }

        prevValue = currentValue;
        prevTime = currentTime;
        return sign * currentValue;
    }
}
