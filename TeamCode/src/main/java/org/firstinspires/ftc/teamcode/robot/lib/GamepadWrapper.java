package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;

/***
 * Class to wrap a gamepad object to allow Object Oriented field access.
 * This allows mapping between ENUM values and gamepad fields.
 */
public class GamepadWrapper {
    private Gamepad gamepad;
    private Class gamepadClass = Gamepad.class; // used for reflection to get the fields
    private Telemetry telemetry;

    /***
     * Constructor for the @GamepadWrapper
     * @param gamepad the gamepad that this wraps (around)
     */
    public GamepadWrapper(Gamepad gamepad, Telemetry telemetry){
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    /***
     * Gets the value of the button @buttonName from the gamepad
     * @param buttonName the button to get the value of [eg "a" or "x"]
     * @return true if the button is pressed
     */
    public boolean getButton(String buttonName) {
        try {
            Field button = gamepadClass.getField(buttonName);
            button.setAccessible(true);

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
            joystick.setAccessible(true);

            return Double.valueOf((float) joystick.get(gamepad));
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
            String joystickX = stick + "_x";
            String joystickY = stick + "_y";
            double x = this.getJoystick(joystickX);
            double y = 0 - this.getJoystick(joystickY);

            return Math.atan2(x, y);
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
        String joystickX = stick + "_x";
        String joystickY = stick + "_y";

        double x = this.getJoystick(joystickX);
        double y = 0 - this.getJoystick(joystickY);

        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /***
     * Get the vector for the joystick of the gamepad.
     * @param stick the stick to get the vector for (left_stick or right_stick)
     * @return the vector of the joystick
     */
    public Vector getStickVector(String stick){
        return new Vector(this.getStickMagnitude(stick), this.getStickAngle(stick));
    }
}
