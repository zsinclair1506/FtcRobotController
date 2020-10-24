package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

/***
 * this class is for the loader mechanism on the robot
 */
public class Loader extends Mechanism {
    private Servo loaderMotor;

    /***
     * Loader constructor
     * @param map hardware map of the robot
     */
    public Loader(HardwareMap map){
        this.loaderMotor = map.get(Servo.class, "loaderMotor");
    }

    /***
     * Load a ring onto the loader
     */
    public void load(){

    }

    /***
     * Lower the loader from shooting angle/height to loading height.
     */
    public void lower(){

    }

    /***
     * Raise the loader from loading height to shooting angle/height
     */
    public void raise(){

    }
}
