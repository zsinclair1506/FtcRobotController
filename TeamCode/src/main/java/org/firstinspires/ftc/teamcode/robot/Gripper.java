package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.GripperPosition;

/***
 * The class is controlling the Gripper mechanism
 */
public class Gripper extends Mechanism {

    /***
     * Gripper constructor. Maps servos for grippers.
     * @param map the hardware map to get the servos from.
     */
    public Gripper(HardwareMap map, Telemetry telemetry){
        super(telemetry);
    }

    /***
     * Closes the Gripper mechanism
     */
    public void closeGripper() {

    }

    /***
     * Opens the gripper mechanism
     */
    public void openGripper(){

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

    /***
     * Setting the position of the gripper from Ring or Wobble Goal
     * @param position to move the arm to
     */
    public void moveToPos(GripperPosition position){

    }
}
