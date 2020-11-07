package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.mapping.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Odometry;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;

import java.util.HashMap;

/***
 * The actions that all drivebases must do as well as some common access methods.
 */
public abstract class DriveBase extends Mechanism {
    private HashMap<String, DcMotor> motors = new HashMap<>();
    private HashMap<String, Double> rotateMotorPowers = new HashMap<>();
    private HashMap<String, Double> strafeMotorPowers = new HashMap<>();
    private HashMap<String, Double> drivePower = new HashMap<>();
    private Odometry odometry;

    /***
     * Constructor for the DriveBase that makes telemetry available to all DriveBases.
     * @param telemetry the telemetry for the DriveBases
     */

    public DriveBase(HardwareMap map, Telemetry telemetry, Robot robot){
        super(telemetry, robot);
        this.odometry = new Odometry(map, telemetry);
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
     * @param distance the distance to drive from current location
     * @param angle the angle to drive the robot (each drivebase will implement this differently)
     */
    public abstract void driveDistance(double distance, double angle);

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
     *
     * Normalises the motor powers with the maximum value of 1 when input to the motors while
     * preserving the desired ratios of power.
     */
    protected void motorNormalise() {
        double maxValue = 0;

        // calculate the max value
        for(String motor : this.getMotors().keySet()){
            maxValue = Math.max(maxValue, Math.abs(this.getDrivePowers().get(motor)));
        }

        // only scale the value if it is above 1. This will be most of the time.
        if(maxValue > 1) {
            for (String motor : this.getMotors().keySet()) {
                this.setDrivePower(motor, this.getDrivePowers().get(motor) / maxValue);
            }
        }
    }

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
        for(String motor : this.getMotors().keySet()){
            double strafe = this.strafeMotorPowers.get(motor) == null
                    ? 0 : this.strafeMotorPowers.get(motor);
            double rotate = this.rotateMotorPowers.get(motor) == null
                    ? 0 : this.rotateMotorPowers.get(motor);
            this.setDrivePower(motor, strafe + rotate);
        }
    }

    /***
     * Combine motor powers. @see DriveBase#combineMotorPower
     */
    protected void byOurPowersCombined(){
        this.combineMotorPower();
    }

    /***
     * Gets the odometry.
     * @return the odometry
     */
    public Odometry getOdometry() {
        return this.odometry;
    }
}
