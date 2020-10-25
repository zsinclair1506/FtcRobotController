package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

/***
 * The class controlling the wobble goal arm mechanism
 */
public class WobbleArm extends Mechanism {
   //private ?? motors
    private Gripper gripper;

    /***
     * Wobblearm constructor
     * @param map hardware map of the robot
     */
    public WobbleArm(HardwareMap map){
        // assign motors here
        this.gripper = new Gripper(map);
    }

    /***
     * Rotates the arm to a set @angle with a power
     * @param angle the angle to move the arm to [0-1]
     * @param power the power with which to move the arm [0-1]
     */
   public void rotate(double angle, double power) {
        // this will need threading
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
   public void gripperOpen(){
       this.gripper.openGripper();
   }

    /***
     * Closes the gripper on the arm
     */
   public void gripperClose(){
       this.gripper.closeGripper();
   }

    /***
     * Stores the arm in initial condition
     */
   public void storeArm(){
       rotate(0, 1);
       gripperOpen();
   }

   public void moveArm(double angle, double power){

   }

   public void moveTo(double x, double y, double z) throws IllegalArgumentException{
       throw new IllegalArgumentException("message");
   }

    public void gripperTiltUp() {
       this.gripper.tiltUp();
    }

    public void gripperTiltDown() {
       this.gripper.tiltDown();
    }
}
