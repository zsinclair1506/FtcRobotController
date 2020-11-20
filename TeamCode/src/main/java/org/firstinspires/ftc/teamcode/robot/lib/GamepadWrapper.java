package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.robot.mapping.GamepadButtonMap;

import java.lang.reflect.Field;
import java.util.HashMap;

/***
 * Class to wrap a gamepad object to allow Object Oriented field access.
 * This allows mapping between ENUM values and gamepad fields.
 */
public class GamepadWrapper {
    private Gamepad gamepad;
    private Class gamepadClass = Gamepad.class; // used for reflection to get the fields
    private HashMap<GamepadButtonMap, ElapsedTime> debounceTimers = new HashMap<>();
    private HashMap<GamepadButtonMap, Boolean> debounce = new HashMap<>();
    private static final double DEBOUNCE_TIME_MS = 250;

    /***
     * Constructor for the @GamepadWrapper. Sets up the debouncing based on which gamepad is
     * supplied.
     * @param gamepad the gamepad that this wraps (around)
     */
    public GamepadWrapper(Gamepad gamepad){
        this.gamepad = gamepad;
        if(gamepad.getUser() == GamepadUser.ONE){ // Driver Gamepad
            for(GamepadButtonMap.DriverGamepad button : GamepadButtonMap.DriverGamepad.values()){
                if(button.getDebounce()){
                    this.addDebounce(button);
                }
            }
        }
        else{ // Operator Gamepad
            for(GamepadButtonMap.OperatorGamepad button : GamepadButtonMap.OperatorGamepad.values()){
                if(button.getDebounce()){
                    this.addDebounce(button);
                }
            }
        }
    }

    /***
     * Add a debounce for a given button.
     * @param button the button to add a debounce for
     */
    private void addDebounce(GamepadButtonMap button){
        this.debounceTimers.put(button, new ElapsedTime());
        this.debounce.put(button, false);
    }

    /***
     * Gets the value of the button from the gamepad. This checks for debouncing and provides
     * debounce functionality where required by the GamepadButtonMap.
     * @param button the button to get the value of
     * @return true if the button is pressed
     */
    public boolean getButton(GamepadButtonMap button) {
        boolean read = false;

        if(this.debounce.containsKey(button)){
            this.debounceReset(button);

            if(!button.getDebounce()) {
                this.debounce.put(button, true);
                this.debounceTimers.get(button).reset();

                read = true;
            }
        }
        else{
            read = true;
        }

        if(read) {
            try {
                Field buttonField = gamepadClass.getField(button.getButtonName());
                buttonField.setAccessible(true);

                return (Boolean) buttonField.get(gamepad);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                //ignore exceptions
            }
        }

        return false;
    }

    /***
     * Gets the value of the joystick axis from the gamepad.
     * @param joystick the joystick axis to get the value of
     * @return double value of the joystick axis
     */
    public double getJoystick(GamepadButtonMap joystick) {
        try {
            Field joystickField = gamepadClass.getField(joystick.getButtonName());
            joystickField.setAccessible(true);

            return Double.valueOf((float) joystickField.get(gamepad));
        } catch (NoSuchFieldException | IllegalAccessException e) {
            //ignore exceptions
        }

        return 0.0;
    }

    /***
     * Gets the value of the trigger @trigger from the gamepad.
     * @param trigger the trigger to get the value of
     * @return double value of the trigger
     */
    public double getTrigger(GamepadButtonMap trigger){
        return getJoystick(trigger);
    }

    /***
     * Gets whether the trigger is pressed or not. This checks for debouncing and provides debounce
     * functionality where required by the GamepadButtonMap.
     * @param trigger the trigger to get the value of
     * @return true if the trigger is 'pressed' (> 0.8)
     */
    public boolean getTriggerBool(GamepadButtonMap trigger){
        boolean read = false;

        if(this.debounce.containsKey(trigger)){
            this.debounceReset(trigger);

            if(!trigger.getDebounce()) {
                this.debounce.put(trigger, true);
                this.debounceTimers.get(trigger).reset();

                read = true;
            }
        }
        else{
            read = true;
        }

        if(read) {
            return (getJoystick(trigger) > 0.8);
        }

        return false;
    }

    /***
     * Get the angle the stick is being pressed in. (0 is +ve X)
     * @param stick the stick to get the angle of (left_stick or right_stick)
     * @return the angle of the stick
     */
    private double getStickAngle(GamepadButtonMap stick) {
        double x = this.getJoystick(stick);
        double y = 0 - this.getJoystick(stick);

        return Math.atan2(y, x);
    }

    /***
     * Get the magnitude of the stick displacement (for power calculations)
     * @param stick the stick to get the magnitude of displacement for
     * @return the magnitude of the displacement
     */
    private double getStickMagnitude(GamepadButtonMap stick){
        double x = this.getJoystick(stick);
        double y = 0 - this.getJoystick(stick);

        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /***
     * Get the vector for the joystick of the gamepad.
     * @param stick the stick to get the vector for (left_stick or right_stick)
     * @return the vector of the joystick
     */
    public Vector getStickVector(GamepadButtonMap stick){
        return new Vector(this.getStickMagnitude(stick), this.getStickAngle(stick));
    }

    /***
     * Check the debounce timer for the button and reset if it is passed the preset time.
     * @param button the button to check the debounce for
     */
    private void debounceReset(GamepadButtonMap button){
        if (debounceTimers.get(button).milliseconds() > DEBOUNCE_TIME_MS) {
            this.debounce.put(button, false);
        }
    }
}
