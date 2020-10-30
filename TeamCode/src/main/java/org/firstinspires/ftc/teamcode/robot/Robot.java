package org.firstinspires.ftc.teamcode.robot;

import java.util.HashMap;

/***
 * An abstract class containing mechanisms and a drivebase for the robot
 */
public abstract class Robot {

    /***
     * mechasnisms - A hashmap of mechanisms and their names
     */
    protected HashMap<String, Mechanism> mechanisms = new HashMap<>();

    /***
     * driveBase - the field for the drivebase of the robot
     */
    protected DriveBase driveBase;

}
