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
