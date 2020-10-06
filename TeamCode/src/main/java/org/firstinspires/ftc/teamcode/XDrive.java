package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.RotationDirection;


public class XDrive extends DriveBase {

    private static XDrive instance = null;

    /***
     * Messy singleton implementation.
     * @param map the hardware map of the robot/phone/expansion hub.
     * @return the existing instance of the X DriveBase or a new instance on first time.
     */
    public static XDrive getInstance(HardwareMap map) {
        if(instance == null) {
            instance = new XDrive(map);
        }

        return instance;
    }

    /***
     * Private constructor that creates the motor map for each motor on the robot.
     * @param map the hardware map of the robot/phone/expansion hub.
     */
    private XDrive(HardwareMap map) {

        this.motors.put("frontLeft", map.get(DcMotor.class, "frontLeft"));
        this.motors.put("frontRight", map.get(DcMotor.class, "frontRight"));
        this.motors.put("backRight", map.get(DcMotor.class, "backRight"));
        this.motors.put("backLeft", map.get(DcMotor.class, "backLeft"));
    }

    /***
     *
     * @param angle
     * @param power
     */
    @Override
    public void DrivePower(float angle, float power) {

    }

    /***
     *
     * @param angle
     * @param distance
     */
    @Override
    public void DriveDistance(float angle, float distance) {

    }

    /***
     *
     * @param angle
     * @param power
     */
    @Override
    public void RotateAngle(float angle, float power) {

    }

    /***
     *
     * @param direction
     * @param power
     */
    @Override
    public void RotateDirection(RotationDirection direction, float power) {

    }
}