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
    SHOOTER_SHOOT("left_trigger"),

    /***
     *
     */
    SHOOTER_FEED_ME("dpad_left"),

    /***
     * Gamepad: 2
     */
    CONVEYOR_RUN("right_bumper"),

    /***
     *
     */
    GRIPPER_ANGLE_FOR("dpad_right");

    /***
     *
     */

    GamepadButtons(String buttonName){
        this.buttonName = buttonName;
    }


    private String buttonName;

    public String getButtonName(){
        return buttonName;
    }


}
