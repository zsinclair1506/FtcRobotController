package org.firstinspires.ftc.teamcode.robot.mapping;

/***
 * Class to define all the gamepad buttons.
 *
 *
 *       ___LTrigger__                   __RTrigger__
 *      /             \                 /            \
 *     /               ----------------          Y    \
 *    |   DPAD                                X    B   \
 *    |                                         A      |
 *    |       Left Stick         Right Stick           |
 *    |                  ________                     |
 *    |                 |        |                   |
 *    \                |         |                  |
 *     \______________/          \_________________/
 * GamePad 1:
 *      Robot rotate - right stick x
 *      Robot strafe - left stick
 *      Shooter Feed Me - left trigger
 *      Shooter Shoot - right trigger
 * GamePad 2:
 *      Conveyor run - LTrigger
 *      Gripper close - dpad left
 *      Gripper lower - dpad down
 *      Gripper open - dpad right
 *      Gripper raise - dpad up
 *      Arm up/down/left/right - right stick
 *      Arm in/out - left stick y
 *      Intake down - a
 *      Intake grab - x
 *      Intake release - b
 *      Intake up - y
 *      Intake switch positions - RTrigger
 */
public enum GamepadButtons {

    /***
     * Gamepad: 1
     */
    SHOOTER_SHOOT("right_trigger"),

    /***
     * Gamepad: 1
     */
    SHOOTER_FEED_ME("dpad_left"),

    /***
     * Gamepad: 1
     */
    ROBOT_STRAFE("left_stick"),

    /***
     * Gamepad: 1
     */
    ROBOT_ROTATE("right_stick"),

    /***
     * Gamepad: 2
     */
    ARM_MOTION("right_stick"),

    /***
     * Gamepad: 2
     */
    ARM_IN_OUT("left_stick"),

    /***
     * Gamepad: 2
     */
    INTAKE_GRAB("x"),

    /***
     * Gamepad: 2
     */
    INTAKE_RELEASE("b"),

    /***
     * Gamepad: 2
     */
    INTAKE_UP("y"),

    /***
     * Gamepad: 2
     */
    INTAKE_SWITCH_POSITIONS("right_trigger"),

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
    CONVEYOR_RUN("right_trigger"),

    /***
     * Gamepad: 2
     */
    GRIPPER_OPEN("dpad_right"),

    /***
     * Gamepad: 2
     */
    INTAKE_DOWN("a"),

    /***
     * Gamepad: 2
     */
    GRIPPER_CLOSE("b");

    private final String buttonName;
    /***
     *
     * @param bName
     */
    GamepadButtons(String bName){
        buttonName = bName;
    }


    /***
     * Getter for the buttonName String
     * @return buttonName, buttonName is 'gotten' and returned
     */
    public String getButtonName(){
        return buttonName;
    }

}
