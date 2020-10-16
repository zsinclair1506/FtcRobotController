package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

/***
 * The class controlling the wobble goal arm mechanism
 */
public class WobbleArm extends Mechanism {
   //private ?? motors
    private Servo gripperServo;

    /***
     * Wobblearm constructor
     * @param map hardware map of the robot
     */
    public WobbleArm(HardwareMap map){
        this.gripperServo = map.get(Servo.class, "gripperservo");
    }

    /***
     * Rotates the arm to a set @angle with a power
     * @param angle the angle to move the arm to [0-180]
     * @param power the power with which to move the arm [0-1]
     */
   public void rotate(double angle, double power) {

   }

    /***
     * Rotates the arm in a direction at a power
     * @param direction the rotation direction the arm will rotate in
     *                  (Clockwise or Counter Clockwise)
     * @param power the power to rotate the arm with [0-1]
     */
   public void rotate(RotationDirection direction, double power){

   }

    /***
     * Opens the gripper on the arm
     */
   public void openGripper(){

   }

    /***
     * Closes the gripper on the arm
     */
   public void CloseGripper(){
       /*
            initial planning stage for the gripper
            servo only supports set position, does not spin continuous
        */
   }

    /***
     * Stores the arm in initial condition
     */
   public void storeArm(){
       rotate(0, 1);
       openGripper();
   }
}
