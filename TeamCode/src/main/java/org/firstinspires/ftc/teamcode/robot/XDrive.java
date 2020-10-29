package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;


public class XDrive extends DriveBase {


    /***
     *
     * @param map the hardware map of the robot/phone/expansion hub.
     */
    public XDrive (HardwareMap map) {
        super(map);

        this.motors.put("frontLeft", map.get(DcMotor.class, "frontLeft"));
        this.motors.put("frontRight", map.get(DcMotor.class, "frontRight"));
        this.motors.put("backRight", map.get(DcMotor.class, "backRight"));
        this.motors.put("backLeft", map.get(DcMotor.class, "backLeft"));
    }

    /***
     * Drive the robot at an angle at a power.
     * @param angle the angle to drive the robot in, 0 is forward. This base slides
     * @param power the power with which to move the robot
     */
    @Override
    public void drivePower(double angle, double power) {
        this.motors.get("frontLeft").setPower(power * Math.cos(angle + 3*Math.PI/4));
        this.motors.get("frontRight").setPower(power * Math.cos(angle + Math.PI/4));
        this.motors.get("backRight").setPower(power * (0 - Math.cos(angle + 3*Math.PI/4)));
        this.motors.get("backLeft").setPower(power * (0 - Math.cos(angle + Math.PI/4)));
    }

    /***
     * Drive the robot along a set vector.
     * @param driveVector the vector to drive the robot along
     */
    @Override
    public void drivePower(Vector driveVector) {
        Vector drive = this.motorNormalise(driveVector);

        this.motors.get("frontLeft").setPower(drive.getMagnitude()
                * Math.cos(drive.getAngleBetween(Vector.X_2) - Math.PI/4));
        this.motors.get("frontRight").setPower(drive.getMagnitude()
                * Math.cos(drive.getAngleBetween(Vector.X_2) + Math.PI/4));
        this.motors.get("backRight").setPower(drive.getMagnitude()
                * Math.cos(drive.getAngleBetween(Vector.X_2) - Math.PI/4));
        this.motors.get("backLeft").setPower(drive.getMagnitude()
                * Math.cos(drive.getAngleBetween(Vector.X_2) + Math.PI/4));

    }

    /***
     * Drive the robot a set distance at a certain angle. (Meant to be run in a loop)
     * @param angle the angle to drive the robot in, 0 is forward. This base slides
     * @param distance the distance to drive from current location
     */
    @Override
    public void driveDistance(double angle, double distance) {

    }

    /***
     * Rotate the robot by a certain angle at a certain power.
     * @param angle the angle to rotate through (clockwise is positive)
     * @param power the power to rotate the drivebase with
     */
    @Override
    public void rotateAngle(double angle, double power) {

    }

    /***
     * Rotate the robot a certain direction at a certain power (meant to be run in a loop)
     * @param direction the rotation direction of the robot
     *                  (top down. CLOCKWISE or COUNTER_CLOCKWISE)
     * @param power the power with which to rotate the robot
     */
    @Override
    public void rotateDirection(RotationDirection direction, double power) {

    }

    /***
     * Creates a vector with the maximum value of 1 when input to the motors while preserving the
     * initial vector
     * @param vector the original vector to motorNormalise
     * @return the new vector with magnitude large enough to make the max motor value 1
     */
    @Override
    public Vector motorNormalise(Vector vector) {
        vector = vector.getUnit();
        double[] values = new double[4];
        values[0] = Math.cos(vector.getAngleBetween(Vector.X_2) - Math.PI/4);
        values[1] = Math.cos(vector.getAngleBetween(Vector.X_2) + Math.PI/4);
        values[2] = 0 - Math.cos(vector.getAngleBetween(Vector.X_2) - Math.PI/4);
        values[3] = 0 - Math.cos(vector.getAngleBetween(Vector.X_2) + Math.PI/4);

        double maxValue = values[0];
        for(int i = 1; i < values.length; i++){
            if(values[i] > maxValue){
                maxValue = values[i];
            }
        }

        return vector.scale(1 / maxValue);
    }
}
