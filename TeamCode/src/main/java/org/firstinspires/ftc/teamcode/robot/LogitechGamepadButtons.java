package org.firstinspires.ftc.teamcode.robot;

/***
 * Class to define all the gamepad buttons.
 */
public enum LogitechGamepadButtons {

    /***
     * 
     */
    START_INTAKE,

    /***
     *
     */
    STOP_INTAKE,

    /***
     *
     */
    EXTEND_ARM,

    /***
     *
     */
    RETRACT_ARM,

    /***
     *
     */
    STOW_ARM,

    OPEN_CLAW,

    /***
     *
     */
    CLOSE_CLAW,

    /***
     * test
     */
    SORT_LEFT,

    /***
     *
     */
    SORT_RIGHT,

    /***
     *
     */
    SHOOT_RIGHT,

    /***
     *
     */
    SHOOT_LEFT,

    /***
     *
     */
    SWITCH_LEFT,

    /***
     *
     */
    SWITCH_RIGHT;

    /***
     *
     */
    RUN_CONVEYOR,

    private int buttonNum;

    /***
     * ???
     * @param buttonNum
     */
    private LogitechGamepadButtons(int buttonNum)
    {
        setButtonNum(buttonNum);
    }

    /***
     *
     * @param buttonNum
     * @return
     */
    public int getButtonNum(int buttonNum) {
        return buttonNum;
    }

    /***
     *
     * @param buttonNum
     */
    public void setButtonNum(int buttonNum) {
        this.buttonNum = buttonNum;
    }
}
