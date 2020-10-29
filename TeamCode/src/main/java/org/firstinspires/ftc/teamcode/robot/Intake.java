package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

/***
 * this class is for the intake mechanism on the robot
 */

public class Intake extends Mechanism {
    //Private ?? motors

    /***
     * Intake constructor
     * @param map the hardware map of the robot
     */
    public Intake(HardwareMap map){

    }

    /***
     * runs the intake
     */
    public void runIntake(){
        runIntake(1);
    }

    /***
     * runs the intake with a certain @power
     * @param power defines the speed of the intake
     */
    public void runIntake(double power){

    }

    /***
     * spits out ring from the intake
     */
    public void runEject(){
        runEject(1);
    }

    /***
     * runs the eject with a certain @power
     * @param power defines the speed of the intake
     */
    public void runEject(double power){

    }

    /***
     * extends the intake mechanism past the frame of the robot
     */
    public void extend(){

    }

    /***
     * retracts the intake mechanism into the frame of the robot
     */
    public void retract(){

    }

    /***
     *
     */
    public void stopIntake() {

    }
}
