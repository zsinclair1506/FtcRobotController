package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

/***
 * Class to define all the gamepad buttons.
 */
public enum GamepadButtons {

    /***
     *
     */
    START_INTAKE(""),

    /***
     *
     */
    STOP_INTAKE(""),

    /***
     *
     */
    EXTEND_ARM(""),

    /***
     *
     */
    RETRACT_ARM(""),

    /***
     *
     */
    STOW_ARM(""),

    /***
     *
     */
    OPEN_CLAW(""),

    /***
     *
     */
    CLOSE_CLAW(""),

    /***
     * test
     */
    SORT_LEFT(""),

    /***
     *
     */
    SORT_RIGHT(""),

    /***
     *
     */
    SHOOT_RIGHT(""),

    /***
     *
     */
    SHOOT_LEFT(""),

    /***
     *
     */
    SWITCH_LEFT(""),

    /***
     *
     */
    SWITCH_RIGHT(""),

    /***
     *
     */
    RUN_CONVEYOR("");

    private GamepadButtons(String buttonName){
        this.buttonName = buttonName;
    }


    private String buttonName;

    public String getButtonName(){
        return buttonName;
    }


}
