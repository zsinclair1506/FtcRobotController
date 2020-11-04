package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

/***
 * The class is controlling the Gripper mechanism
 */
public class Gripper extends Mechanism {
    private Servo clawServo;

    public enum GripperPosition {
        RING,
        WOBBLE_GOAL,
        OPEN,
        CLOSED,
        ;
    }

    /***
     * Gripper constructor. Maps servos for grippers.
     * @param map the hardware map to get the servos from.
     */
    public Gripper(HardwareMap map, Telemetry telemetry, Robot robot){
        super(telemetry, robot);
        this.clawServo = map.get(Servo.class, MotorMap.GRIPPER_SERVO.getMotorName());
    }

    /***
     * Closes the Gripper mechanism
     */
    public void closeGripper() {
        double currentPos = this.clawServo.getPosition();
        this.clawServo.setPosition(currentPos -= 0.05);
    }

    /***
     * Opens the gripper mechanism
     */
    public void openGripper() {
        double currentPos = this.clawServo.getPosition();
        this.clawServo.setPosition(currentPos += 0.05);
    }

    /***
     * Gets if the Gripper is open or not
     * @return true if Gripper is open
     */
    public boolean getIsOpen(){
        return true;
    }

    /***
     * Tilts the Gripper mechanism up
     */
    public void tiltUp(){

   }

    /***
     * Tilts the Gripper mechansim down
     */
   public void tiltDown(){

   }

   public void gripperPosition(GripperPosition gPos){
       switch(gPos){
           case OPEN:
               clawServo.setPosition(0);
               break;
           case RING:
               clawServo.setPosition(0);
               break;
           case WOBBLE_GOAL:
               clawServo.setPosition(0);
               break;
           case CLOSED:
               clawServo.setPosition(0);
               break;
       }
   }

    /***
     * Setting the position of the gripper from Ring or Wobble Goal
     * @param position to move the arm to
     */
    public void moveToPos(GripperPosition position){

    }
}
