package org.firstinspires.ftc.teamcode;

/***
 * The class responsible for the running of the conveyor mechanism on the robot
 */
public class Conveyor extends Mechanism {
    //private ?? motor
    private Boolean ringPresent = false;

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
    public void Convey(double power){

    }

    /***
     * Conveys rings from the intake to the shooting mechanism with a preset power
     */
    public void Convey(){
        Convey(1);
    }

}
