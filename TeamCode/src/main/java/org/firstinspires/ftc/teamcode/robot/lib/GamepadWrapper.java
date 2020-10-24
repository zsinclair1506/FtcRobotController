package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.lang.reflect.Field;

/***
 * Class to wrap a gamepad object to allow Object Oriented field access.
 * This allows mapping between ENUM values and gamepad fields.
 */
public class GamepadWrapper {
    private Gamepad gamepad;
    private Class gamepadClass = Gamepad.class; // used for reflection to get the fields

    /***
     * Constructor for the @GamepadWrapper
     * @param gamepad the gamepad that this wraps (around)
     */
    public GamepadWrapper(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    /***
     * Gets the value of the button @buttonName from the gamepad
     * @param buttonName the button to get the value of [eg "a" or "x"]
     * @return true if the button is pressed
     */
    public boolean getButton(String buttonName) {
        try {
            Field button = gamepadClass.getField(buttonName);
            return (Boolean) button.get(gamepad);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            //ignore exceptions, return false
            return false;
        }
    }

    /***
     * Gets the value of the joystic @joystickAxis from the gamepad
     * @param joystickAxis the joystick axis to get the value of [eg "left_stick_y"]
     * @return double value of the joystick axis
     */
    public double getJoystick(String joystickAxis){
        try {
            Field joystick = gamepadClass.getField(joystickAxis);
            return (double) joystick.get(gamepad);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            //ignore exceptions, return 0
            return 0.0f;
        }
    }

    /***
     * Gets the value of the trigger @trigger from the gamepad
     * @param trigger the trigger to get the value of
     * @return double value of the trigger
     */
    public double getTrigger(String trigger){
        return getJoystick(trigger);
    }

    /***
     * Assume stick forward gives 0 angle and clockwise is +ve
     * @param stick the stick to get the angle of (left or right)
     * @return the angle of the stick (0 forward, +ve clockwise)
     */
    public double getStickAngle(String stick){
        if (stick.contains("left") | stick.contains("right")) {
            try {
                String joystickX = stick + "_stick_x";
                String joystickY = stick + "_stick_y";
                Field joystick_X = gamepadClass.getField(joystickX);
                Field joystick_Y = gamepadClass.getField(joystickY);

                double x = (double) joystick_X.get(joystickX);
                double y = (double) joystick_Y.get(joystickY);
                double angle = Math.atan(y / x);

                if (x > 0)
                {
                    // Q1, Q4
                    return Math.PI/2 - angle;
                }
                else{
                    // Q2, Q3
                    return 0 - (Math.PI/2 + angle);
                }
            } catch (NoSuchFieldException | IllegalAccessException e) {
                //ignore exceptions, return 0
                return 0;
            }
        }
        else{
            return 0;
            }
    }
}
