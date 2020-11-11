package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

/***
 * An abstract class containing mechanisms and a drivebase for the robot
 */
public abstract class Robot {

    protected Telemetry telemetry;

    /***
     * mechasnisms - A hashmap of mechanisms and their names
     */
    protected HashMap<String, Mechanism> mechanisms = new HashMap<>();

    /***
     * driveBase - the field for the drivebase of the robot
     */
    protected DriveBase driveBase;

    protected Robot(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /***
     * Adds a mechanism to the mechanisms collection
     * @param name the unique name of the mechanism to add (the key)
     * @param mechanism the mechanism to add (the value)
     */
    protected void addMechanism(String name, Mechanism mechanism){
        this.mechanisms.put(name, mechanism);
    }

    /***
     * Gets a mechanism from the mechanism collection
     * @param name the name of the mechanism (key)
     * @return the mechanism if it exists or null
     */
    protected Mechanism getMechanism(String name){
        return this.mechanisms.get(name);
    }

    /***
     * Sets the drivebase of the robot
     * @param driveBase the drivebase to set for the robot
     */
    protected void setDriveBase(DriveBase driveBase){
        this.driveBase = driveBase;
    }

    /***
     * Gets the current drivebase of the robot
     * @return the drivebase the robot is using
     */
    protected DriveBase getDriveBase(){
        return this.driveBase;
    }

}
