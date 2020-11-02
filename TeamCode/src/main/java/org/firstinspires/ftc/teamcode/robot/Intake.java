package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.IntakePosition;

/***
 * This class is for the intake mechanism on the robot. This mechanism has linear motion bottom
 * to top. Has a pop out and pop in action for gripping the ring. It also has rotation to place
 * ring in the passive hopper.
 */
public class Intake extends Mechanism {
    //private Servo rotationServo;
    //private CRServo vertServo;
    //private Servo grabServo;

    /***
     * Intake constructor
     * @param map the hardware map of the robot
     */
    public Intake(HardwareMap map, Telemetry telemetry, Robot robot){
        //map servos from hardware map
        super(telemetry, robot);
    }

    /***
     * lifts the intake mechanism up from the floor
     */
    public void lift(){

    }

    /***
     * causes the intake mechanism to grab a ring
     */
    public void grab(){

    }

    /***
     * causes the intake mechanism to release a ring
     */
    public void release(){

    }

    /***
     * lowers the intake mechanism to the floor
     */
    public void lower(){

    }

    /***
     * stop all motion on the intake mechanism
     */
    public void stop(){

    }

    /***
     * go through the actions to drop a ring off at the hopper
     */
    public void dropOff(){
        /*
        Will likely have to be a thread since this series of actions will take time and we want
        to continue to move around and do other things. Will have to check for thread running in
        each of the other methods to make sure we're not accessing data concurrently.
         */
    }

    /***
     * rotates the intake mechanism to a set location
     * @param position the position to rotate the intake to (INTAKE or DROP_OFF)
     */
    public void rotate(IntakePosition position){

    }
}
