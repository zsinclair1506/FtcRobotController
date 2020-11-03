package org.firstinspires.ftc.teamcode.robot.mapping;

/***
 * A map of the mechanism names (for consistency and easy of use).
 */
public enum MechanismMap {
    CONVEYOR("conveyor"),
    GRIPPER("gripper"),
    INTAKE("intake"),
    LOADER("loader"),
    SHOOTER("shooter"),
    XDRIVE("xdrive"),
    WOBBLE_GOAL_ARM("wobblegoal"),
    ;

    private final String mechanismName;

    /***
     * Constructor setting the name of the mechanism.
     * @param name
     */
    MechanismMap(String name){
        this.mechanismName = name;
    }

    /***
     * Gets the name of the mechanism.
     * @return the mechanism name
     */
    public String getName(){
        return this.mechanismName;
    }
}
