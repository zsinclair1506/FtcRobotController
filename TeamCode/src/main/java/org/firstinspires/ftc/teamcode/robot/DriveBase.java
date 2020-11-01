package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;

import java.util.HashMap;

/***
 * The actions that all drivebases must do.
 */
public abstract class DriveBase {
    private HashMap<String, DcMotor> motors = new HashMap<>();
    private HashMap<String, Double> motorPowers = new HashMap<>();
    protected Telemetry telemetry;

    /***
     * Constructor for the DriveBase that makes telemetry available to all DriveBases.
     * @param telemetry the telemetry for the DriveBases
     */
    public DriveBase(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /***
     * Execute the programmed drive mechanic
     */
    public abstract void drive();

    /***
     * Set the motor power for the motors to drive at the set angle. Will need to execute
     * @see DriveBase#drive(). Must be run in a loop.
     * @param angle the angle to drive the robot (each drivebase will implement this differently)
     * @param power the power to drive the robot at [0 - 1]
     */
    public abstract void setStrafe(double angle, double power);

    /***
     * Set the motor power for the motors to drive at the set angle. Will need to execute
     * @see DriveBase#drive(). Must be run in a loop.
     * @param vector the vector to drive the robot at
     */
    public abstract void setStrafe(Vector vector);

    /***
     * Drive the robot a set distance at a certain angle. (Meant to be run in a loop)
     * @param angle the angle to drive the robot (each drivebase will implement this differently)
     * @param distance the distance to drive from current location
     */
    public abstract void driveDistance(double angle, double distance);

    /***
     * Rotate the robot by a certain angle at a certain power.
     * @param angle the angle to rotate through (clockwise is positive)
     * @param power the power to rotate the drivebase with
     */
    public abstract void rotateAngle(double angle, double power);

    /***
     * Set the motor power for the motors to rotate at the set power. Will need to execute
     * @see DriveBase#drive(). Must be run in a loop.
     * @param direction the rotation direction of the robot
     *                  (top down. CLOCKWISE or COUNTER_CLOCKWISE)
     * @param power the power with which to rotate the robot
     */
    public abstract void setRotation(RotationDirection direction, double power);

    /***
     * Set the motor power for the motors to rotate at the set power. Will need to execute
     * @see DriveBase#drive(). Must be run in a loop.
     * @param vector the vector that rotates the robot accordingly
     */
    public abstract void setRotation(Vector vector);

    /***
     * Gets the largest component of the vector for the drive wheels and scales the magnitude
     * to allow a motor to be working at 100% (1) even if the vector would only achieve .7 (should
     * be limited to if the magnitude is close to 1 [0.8 or greater?] otherwise it will always be
     * driving full power)
     * @param vector the vector to normalise
     * @return the 'normalised' vector to the motor power
     */
    public abstract Vector motorNormalise(Vector vector);

    /***
     * Get the motor from the collection of motors for this drivebase.
     * @param motorName the name of the motor to get
     * @return a motor with the given name
     */
    protected DcMotor getMotor(String motorName){
        return this.motors.get(motorName);
    }

    /***
     * Adds a motor to the motor hashmap.
     * @param motorName the name of the motor to add
     * @param motor the motor to add
     */
    protected void addMotor(String motorName, DcMotor motor){
        this.motors.put(motorName, motor);
    }

    /***
     * Get the motor power from the collection of motor powers
     * @param motorName the name of the motor to get the power for
     * @return the power the motor is set to
     */
    protected double getMotorPower(String motorName){
        return this.motorPowers.get(motorName);
    }

    /***
     * Add motor power to the collection of motors
     * @param motorName the name of the motor to set power to
     * @param power the power to set the motor
     */
    protected void setMotorPower(String motorName, double power){
        this.motorPowers.put(motorName, power);
    }

    /***
     * Gets the collection of motors
     * @return the collection of motors
     */
    protected HashMap getMotors(){
        return this.motors;
    }
}
