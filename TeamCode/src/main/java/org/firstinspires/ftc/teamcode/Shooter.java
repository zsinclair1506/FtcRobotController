package org.firstinspires.ftc.teamcode;

/***
 * The class responsible for running and controlling the shooting mechanisms on the robot
 */
public class Shooter extends Mechanism {
    //private ?? motors

    /***
     * Shoots a ring with a @power
     * @param power the power with which to shoot a ring [0-1]
     */
    public void Shoot(double power){

    }

    /***
     * Shoots a ring with a preset power
     */
    public void Shoot() {
    Shoot(1);
    }

    /***
     * Loading the ring into the shooting mechanism
     */
    public void FeedMe(){

    }
}
