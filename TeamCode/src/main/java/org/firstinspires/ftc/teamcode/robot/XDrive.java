package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;


public class XDrive extends DriveBase {

    /***
     *
     * @param map the hardware map of the robot/phone/expansion hub.
     */
    public XDrive (HardwareMap map, Telemetry telemetry) {
        super(telemetry);
        this.addMotor(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName()));
        this.addMotor(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName()));
        this.addMotor(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName()));
        this.addMotor(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName()));
    }

    @Override
    public void drive() {

    }

    /***
     * Drive the robot at an angle at a power.
     * @param angle the angle to drive the robot in, 0 is forward. This base slides
     * @param power the power with which to move the robot
     */
    @Override
    public void setStrafe(double angle, double power) {
        this.getMotor(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName()).setPower(
                power * Math.cos(angle + 3*Math.PI/4));
        this.getMotor(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName()).setPower(
                power * Math.cos(angle + Math.PI/4));
        this.getMotor(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName()).setPower(
                power * (0 - Math.cos(angle + 3*Math.PI/4)));
        this.getMotor(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName()).setPower(
                power * (0 - Math.cos(angle + Math.PI/4)));
    }

    /***
     * Drive the robot along a set vector.
     * @param driveVector the vector to drive the robot along
     */
    @Override
    public void setStrafe(Vector driveVector) {
        Vector drive = driveVector;
        if(driveVector.getMagnitude() > 0.8) {
            drive = this.motorNormalise(driveVector);
        }
        else{
            drive = driveVector;
        }

        this.getMotor(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName()).setPower(
                (-drive.getMagnitude()) * Math.cos(drive.getAngleBetween(Vector.X_2) - Math.PI/4));
        this.getMotor(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName()).setPower(
                (-drive.getMagnitude()) * Math.cos(drive.getAngleBetween(Vector.X_2) + Math.PI/4));
        this.getMotor(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName()).setPower(
                drive.getMagnitude() * Math.cos(drive.getAngleBetween(Vector.X_2) - Math.PI/4));
        this.getMotor(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName()).setPower(
                drive.getMagnitude() * Math.cos(drive.getAngleBetween(Vector.X_2) + Math.PI/4));
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
     * @param power the power with which to rotate the robot
     */
    @Override
    public void setRotation(RotationDirection direction, double power) {
        this.getMotor(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName()).setPower(power);
        this.getMotor(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName()).setPower(power);
        this.getMotor(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName()).setPower(power);
        this.getMotor(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName()).setPower(power);
    }

    /***
     * Rotate the robot according to a vector (meant to be run in a loop)
     * @param vector the vector to rotate the drivebase
     */
    @Override
    public void setRotation(Vector vector) {
        RotationDirection direction = Math.cos(vector.getAngleBetween(Vector.X_2)) < Math.PI/2
                ? RotationDirection.CLOCKWISE : RotationDirection.COUNTER_CLOCKWISE ;
        setRotation(direction, vector.getMagnitude());
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

        return vector.scale(1.0 / maxValue);
    }
}
