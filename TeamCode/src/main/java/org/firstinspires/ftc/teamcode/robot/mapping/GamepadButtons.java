package org.firstinspires.ftc.teamcode.robot.mapping;

/***
 * Class to define all the gamepad buttons.
 *
 *
 *       ___LTrigger__                   __RTrigger__
 *      /   lbumper   \                 /   rbumper  \
 *     /               ----------------          Y    \
 *    |   DPAD         BK        ST           X    B   \
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
 *      Arm in/out - left stick y
 *      Arm motion (up, down, right, left) - right stick
 *      Conveyor run - LTrigger
 *      Gripper close - dpad left
 *      Gripper lower - dpad down
 *      Gripper open - dpad right
 *      Gripper raise - dpad up
 *      Intake cycle - a
 *      Intake grab - x
 *      Intake lift - rbumper
 *      Intake lower - lbumper
 *      Intake release - b
 *      Intake switch positions - RTrigger
 *      Intake stop - back
 */
public enum GamepadButtons {

    /***
     * Gamepad: 1
     */
    ROBOT_ROTATE("right_stick"),

    /***
     * Gamepad: 1
     */
    ROBOT_STRAFE("left_stick"),

    /***
     * Gamepad: 1
     */
    SHOOTER_FEED_ME("dpad_left"),

    /***
     * Gamepad: 1
     */
    SHOOTER_SHOOT("right_trigger"),

    /***
     * Gamepad: 2
     */
    ARM_IN_OUT("left_stick"),

    /***
     * Gamepad: 2
     */
    ARM_MOTION("right_stick"),

    /***
     * Gamepad: 2
     */
    CONVEYOR_RUN("left_trigger"),

    /***
     * Gamepad: 2
     */
    GRIPPER_CLOSE("b"),

    /***
     * Gamepad: 2
     */
    GRIPPER_LOWER("dpad_down"),

    /***
     * Gamepad: 2
     */
    GRIPPER_OPEN("dpad_right"),

    /***
     * Gamepad: 2
     */
    GRIPPER_RAISE("dpad_up"),

    /***
     * Gamepad: 2
     */
    INTAKE_CYCLE("a"),

    /***
     * Gamepad: 2
     */
    INTAKE_LIFT("right_bumper"),

    /***
     * Gamepad: 2
     */
    INTAKE_LOWER("left_bumper"),

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
    INTAKE_STOP("back"),

    /***
     * Gamepad: 2
     */
    INTAKE_SWITCH_POSITIONS("right_trigger"),
    ;

    private final String buttonName;
    /***
     * Constructor for the action/button
     * @param bName the name of the button for the action
     */
    GamepadButtons(String bName){
        this.buttonName = bName;
    }


    /***
     * Get the name of the button for the action
     * @return the name of the button
     */
    public String getButtonName(){
        return this.buttonName;
    }

}
