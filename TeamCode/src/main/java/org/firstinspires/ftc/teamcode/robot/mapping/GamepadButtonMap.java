package org.firstinspires.ftc.teamcode.robot.mapping;

/***
 * Class to define all the gamepad buttons.
 *
 *
 *       ___LTrigger__                   __RTrigger__
 *      /             \                 /            \
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
 *      Intake cycle - start
 *      Intake grab - x
 *      Intake lift - y
 *      Intake lower - a
 *      Intake release - b
 *      Intake switch positions - RTrigger
 *      Intake stop - back
 */
public interface GamepadButtonMap {

    enum DriverGamepad implements GamepadButtonMap {
        /***
         * Gamepad: 1
         */
        ROBOT_ROTATE("right_stick", false),

        /***
         * Gamepad: 1
         */
        ROBOT_STRAFE("left_stick", false),

        /***
         * Gamepad: 1
         */
        SHOOTER_FEED_ME("dpad_left", true),

        /***
         * Gamepad: 1
         */
        SHOOTER_SHOOT("right_trigger", false),

        /***
         * Gamepad: 1
         */
        FEEDER_UP("y", false),

        /***
         * Gamepad: 1
         */
        FEEDER_DOWN("a", false),
        ;

        private final String buttonName;
        private final boolean debounce;

        /***
         * Constructor for the action/button
         * @param bName the name of the button for the action
         */
        DriverGamepad(String bName, boolean debounce){
            this.buttonName = bName;
            this.debounce = debounce;
        }

        public String getButtonName(){
            return this.buttonName;
        }

        public boolean getDebounce(){
            return this.debounce;
        }
    }

    enum OperatorGamepad implements GamepadButtonMap {
        /***
         * Gamepad: 2
         */
        ARM_IN_OUT("left_stick",false),

        /***
         * Gamepad: 2
         */
        ARM_MOTION("right_stick",false),

        /***
         * Gamepad: 2
         */
        CONVEYOR_RUN("left_trigger",false),

        /***
         * Gamepad: 2
         */
        GRIPPER_CLOSE("b",false),

        /***
         * Gamepad: 2
         */
        GRIPPER_LOWER("dpad_down",false),

        /***
         * Gamepad: 2
         */
        GRIPPER_OPEN("dpad_right",false),

        /***
         * Gamepad: 2
         */
        GRIPPER_RAISE("dpad_up",false),

        /***
         * Gamepad: 2
         */
        INTAKE_CYCLE("start",true),

        /***
         * Gamepad: 2
         */
        INTAKE_LIFT("y",false),

        /***
         * Gamepad: 2
         */
        INTAKE_LOWER("a",false),

        /***
         * Gamepad: 2
         */
        INTAKE_GRAB("x",false),

        /***
         * Gamepad: 2
         */
        INTAKE_RELEASE("b",false),

        /***
         * Gamepad: 2
         */
        INTAKE_STOP("back",true),

        /***
         * Gamepad: 2
         */
        INTAKE_SWITCH_POSITIONS("right_trigger",true),
        ;

        private final String buttonName;
        private final boolean debounce;

        /***
         * Constructor for the action/button
         * @param bName the name of the button for the action
         */
        OperatorGamepad(String bName, boolean debounce){
            this.buttonName = bName;
            this.debounce = debounce;
        }

        public String getButtonName(){
            return this.buttonName;
        }

        public boolean getDebounce(){
            return this.debounce;
        }
    }
    ;

    /***
     * Get the name of the button for the action
     * @return the name of the button
     */
    String getButtonName();

    boolean getDebounce();

}
