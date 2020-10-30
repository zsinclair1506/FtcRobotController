package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;

import java.util.HashMap;

/***
 * The actions that all drivebases must do.
 */
public abstract class DriveBase {
    protected HashMap<String, DcMotor> motors = new HashMap<>();
    protected Telemetry telemetry;

    public DriveBase(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
    }

    /***
     * Drive the robot at an angle at a power.
     * @param angle the angle to drive the robot (each drivebase will implement this differently)
     * @param power the power to drive the robot at [0 - 1]
     */
    public abstract void drivePower(double angle, double power);

    /***
     * Drive the robot along a vector.
     * @param vector the vector to drive the robot at
     */
    public abstract void drivePower(Vector vector);

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
     * Rotate the robot a certain direction at a certain power (meant to be run in a loop)
     * @param direction the rotation direction of the robot
     *                  (top down. CLOCKWISE or COUNTER_CLOCKWISE)
     * @param power the power with which to rotate the robot
     */
    public abstract void rotateDirection(RotationDirection direction, double power);

    /***
     * Gets the largest component of the vector for the drive wheels and scales the magnitude
     * to allow a motor to be working at 100% (1) even if the vector would only achieve .7 (should
     * be limited to if the magnitude is close to 1 [0.8 or greater?] otherwise it will always be
     * driving full power)
     * @param vector the vector to normalise
     * @return the 'normalised' vector to the motor power
     */
    public abstract Vector motorNormalise(Vector vector);

}
