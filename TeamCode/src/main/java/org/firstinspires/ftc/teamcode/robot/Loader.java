package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

/***
 * This class is for the loader mechanism on the robot.
 */
public class Loader extends Mechanism {
    private Servo loaderServo;
    private static double LOWER_POSITION = 0;
    private static double UPPER_POSITION = 1;

    /***
     * Loader constructor
     * @param map hardware map of the robot
     */
    public Loader(HardwareMap map, Telemetry telemetry, Robot robot){
        super(telemetry, robot);
        this.loaderServo = map.get(Servo.class, MotorMap.LOADER_SERVO.getMotorName());
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
        this.loaderServo.setPosition(this.LOWER_POSITION);
    }

    /***
     * Raise the loader from loading height to shooting angle/height
     */
    public void raise(){
        this.loaderServo.setPosition(this.UPPER_POSITION);
    }
}
