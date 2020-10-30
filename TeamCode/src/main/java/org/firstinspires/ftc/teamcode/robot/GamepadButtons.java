package org.firstinspires.ftc.teamcode.robot;

/***
 * Class to define all the gamepad buttons.
 */
public enum GamepadButtons {

    /***
     * Gamepad: 2
     */
    GRIPPER_RAISE("dpad_up"),

    /***
     * Gamepad: 2
     */
    GRIPPER_LOWER("dpad_down"),

    /***
     * Gamepad: 2
     */
    ARM_ROTATE("x"),

    /***
     * Gamepad: 2
     */
    GRIPPER_OPEN("a"),

    /***
     * Gamepad: 2
     */
    GRIPPER_CLOSE("b"),

    /***
     * Gamepad: 1
     */
    SHOOTER_SHOOT("right_trigger"),

    /***
     * Gamepad: 1
     */
    SHOOTER_FEED_ME("dpad_left"),

    /***
     * Gamepad: 2
     */
    CONVEYOR_RUN("right_trigger"),

    /***
     * Gamepad: 2
     */
    GRIPPER_ANGLE_FOR("dpad_right"),

    /***
     * Gamepad: 1
     */
    ROBOT_DRIVE("left_stick"),

    /***
     * Gamepad: 1
     */
    ROBOT_ROTATE("right_stick"),

    /***
     * Gamepad: 1
     */
    ROBOT____("y");

    private static String buttonName;

    /***
     *
     * @param buttonName
     */
    GamepadButtons(String buttonName){
        buttonName = buttonName;
    }


    /***
     * Getter for the buttonName String
     * @return buttonName, buttonName is 'gotten' and returned
     */
    public static String getButtonName(){
        return buttonName;
    }


}
