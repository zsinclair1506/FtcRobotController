package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;

import java.util.HashMap;

/***
 * The actions that all drivebases must do as well as some common access methods.
 */
public abstract class DriveBase {
    private HashMap<String, DcMotor> motors = new HashMap<>();
    private HashMap<String, Double> rotateMotorPowers = new HashMap<>();
    private HashMap<String, Double> strafeMotorPowers = new HashMap<>();
    private HashMap<String, Double> drivePower = new HashMap<>();
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
     * Gets the largest component of the motors for the drive wheels and scales the magnitude
     * to allow a motor to be working at 100% (1) even if the motor would only achieve .7 (should
     * be limited to if the magnitude is close to 1 [0.8 or greater?] otherwise it will always be
     * driving full power)
     */
    protected abstract void motorNormalise();

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
     * @param motorName the name of the motor to set the power for
     * @param power the power to set
     */
    protected void setStrafeMotorPower(String motorName, double power){
        this.strafeMotorPowers.put(motorName, power);
    }

    /***
     * Add motor power to the collection of motors
     * @param motorName the name of the motor to set power to
     * @param power the power to set
     */
    protected void setRotateMotorPower(String motorName, double power){
        this.rotateMotorPowers.put(motorName, power);
    }

    /***
     * Gets the collection of motors
     * @return the collection of motors
     */
    protected HashMap<String, DcMotor> getMotors(){
        return this.motors;
    }

    /***
     * Gets the collection of motor drive power
     * @return the motor drive power map
     */
    protected HashMap<String, Double> getDrivePowers(){
        return this.drivePower;
    }

    protected double getDrivePower(String motorName){
        return this.getDrivePowers().get(motorName);
    }

    /***
     * Set the power for the driving of the motors. This is the final power that is applied to the
     * motors. It will need to be between -1 and 1.
     * @param motorName the name of the motor to set the power for
     * @param power the power to set to the motor
     */
    protected void setDrivePower(String motorName, double power){
        this.drivePower.put(motorName, power);
    }

    /***
     * Combines the strafe and rotate powers into drive power.
     */
    protected void combineMotorPower(){
        for(String motor : this.strafeMotorPowers.keySet()){
            this.setDrivePower(motor,
                    this.strafeMotorPowers.get(motor) + this.rotateMotorPowers.get(motor));
        }
    }

    /***
     * Com
     */
    protected void byOurPowersCombined(){
        this.combineMotorPower();
    }
}
