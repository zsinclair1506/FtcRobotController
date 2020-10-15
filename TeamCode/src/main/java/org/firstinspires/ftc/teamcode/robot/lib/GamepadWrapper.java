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
     * @return float value of the joystick axis
     */
    public float getJoystick(String joystickAxis){
        try {
            Field joystick = gamepadClass.getField(joystickAxis);
            return (float) joystick.get(gamepad);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            //ignore exceptions, return 0
            return 0.0f;
        }
    }

    /***
     * Gets the value of the trigger @trigger from the gamepad
     * @param trigger the trigger to get the value of
     * @return float value of the trigger
     */
    public float getTrigger(String trigger){
        return getJoystick(trigger);
    }
}
