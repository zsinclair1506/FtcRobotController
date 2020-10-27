package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

/***
 * The class responsible for running and controlling the shooting mechanisms on the robot
 */
public class Shooter extends Mechanism {
    //private ?? motors

    /***
     * Shooter constructor
     * @param map hardware map of the robot
     */
    public Shooter(HardwareMap map){

    }

    /***
     * Shoots a ring with a @power
     * @param power the power with which to shoot a ring [0-1]
     */
    public void shoot(double power){

    }

    /***
     * Shoots a ring with a preset power
     */
    public void shoot() {
    shoot(1);
    }

    /***
     * Loading the ring into the shooting mechanism
     */
    public void feedMe(){

    }
}
