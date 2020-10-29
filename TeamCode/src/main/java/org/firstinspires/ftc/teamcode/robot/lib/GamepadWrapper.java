package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;

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
     * Gets the value of the joystick @joystickAxis from the gamepad
     * @param joystickAxis the joystick axis to get the value of [eg "left_stick_y"]
     * @return double value of the joystick axis
     */
    public double getJoystick(String joystickAxis){
        try {
            Field joystick = gamepadClass.getField(joystickAxis);
            return (double) joystick.get(gamepad);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            //ignore exceptions, return 0
            return 0;
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
     * Get the angle the stick is being pressed in. (0 is +ve X)
     * @param stick the stick to get the angle of (left_stick or right_stick)
     * @return the angle of the stick
     */
    private double getStickAngle(String stick) {
        if (stick.contains("left_stick") | stick.contains("right_stick")) {
            try {
                String joystickX = stick + "_x";
                String joystickY = stick + "_y";
                Field joystick_X = gamepadClass.getField(joystickX);
                Field joystick_Y = gamepadClass.getField(joystickY);

                double x = (double) joystick_X.get(joystickX);
                double y = (double) joystick_Y.get(joystickY);

                /* this was for 0 forward
                double angle = Math.atan(x / y);
                
                if (y > 0) {
                    // Q1, Q2
                    return angle;
                }
                else {
                    // Q3, Q4
                    return Math.PI + angle;
                }
                */

                return Math.atan2(x, y);

            } catch (NoSuchFieldException | IllegalAccessException e) {
                //ignore exceptions, return 0
                return 0;
            }
        }
        else {
            return 0;
        }
    }

    /***
     * Get the magnitude of the stick displacement (for power calculations)
     * @param stick the stick to get the magnitude of displacement for
     * @return the magnitude of the displacement
     */
    private double getStickMagnitude(String stick){
        try {
            String joystickX = stick + "_x";
            String joystickY = stick + "_y";
            Field joystick_X = gamepadClass.getField(joystickX);
            Field joystick_Y = gamepadClass.getField(joystickY);

            double x = (double) joystick_X.get(joystickX);
            double y = (double) joystick_Y.get(joystickY);

            return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        } catch (NoSuchFieldException | IllegalAccessException e) {
            //ignore exceptions, return 0
            return 0;
        }
    }

    /***
     * Get the vector for the joystick of the gamepad.
     * @param stick the stick to get the vector for (left_stick or right_stick)
     * @return the vector of the joystick
     */
    public Vector getStick(String stick){
        return new Vector(getStickMagnitude(stick), getStickAngle(stick));
    }
}
