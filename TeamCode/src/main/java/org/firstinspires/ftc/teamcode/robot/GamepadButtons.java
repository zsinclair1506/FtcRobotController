package org.firstinspires.ftc.teamcode.robot;

/***
 * Class to define all the gamepad buttons.
 */
public enum GamepadButtons {

    /***
     *
     */
    START_INTAKE("dpad_up"),

    /***
     *
     */
    STOP_INTAKE("dpad_down"),

    /***
     *
     */
    EXTEND_ARM("y"),

    /***
     *
     */
    RETRACT_ARM("x"),

    /***
     *
     */
    OPEN_CLAW("a"),

    /***
     *
     */
    CLOSE_CLAW("b"),

    /***
     * test
     */
    SORT_LEFT("dpad_left"),

    /***
     *
     */
    SORT_RIGHT("dpad_right"),

    /***
     *
     */
    SHOOT_RIGHT("right_trigger"),

    /***
     *
     */
    SHOOT_LEFT("left_trigger"),

    /***
     *
     */
    RUN_CONVEYOR("right_bumper");

    private GamepadButtons(String buttonName){
        this.buttonName = buttonName;
    }


    private String buttonName;

    public String getButtonName(){
        return buttonName;
    }


}
