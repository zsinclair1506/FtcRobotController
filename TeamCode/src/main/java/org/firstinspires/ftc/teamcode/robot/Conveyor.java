package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/***
 * The class responsible for the running of the conveyor mechanism on the robot
 */
public class Conveyor extends Mechanism {
    //private ?? motor
    private Boolean ringPresent = false;

    /***
     * Conveyor constructor
     * @param map the hardware map for the robot
     */
    public Conveyor(HardwareMap map, Telemetry telemetry, Robot robot){
        super(telemetry, robot);
    }

    /***
     * Getter for checking if ring is in conveyor mechanism
     * @return True if ring is present in the mechanism
     */
    public boolean isRingPresent(){
        return ringPresent;
    }

    /***
     * Convey rings from the intake to the shooting mechanism, with a @power [0-1]
     * @param power the power with which to push the rings through the conveyor
     */
    public void convey(double power){

    }

    /***
     * Conveys rings from the intake to the shooting mechanism with a preset power
     */
    public void convey(){
        convey(1);
    }

}
