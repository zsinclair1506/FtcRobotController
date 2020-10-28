package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

import java.util.HashMap;

/***
 * The actions that all drivebases must do.
 */
public abstract class DriveBase {
    protected HashMap<String, DcMotor> motors;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public DriveBase(HardwareMap map) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = map.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    public double getStartHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        return 0;
    }

    /***
     * Drive the robot at an angle at a power.
     * @param angle the angle to drive the robot (each drivebase will implement this differently)
     * @param power the power to drive the robot at [0 - 1]
     */
    public abstract void drivePower(double angle, double power);

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

    public double IMUdriveCorrection(double desiredHeading){

        return 0;
    }
}
