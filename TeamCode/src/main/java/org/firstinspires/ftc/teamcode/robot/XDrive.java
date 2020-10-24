package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;


public class XDrive extends DriveBase {


    /***
     *
     * @param map the hardware map of the robot/phone/expansion hub.
     */
    public XDrive (HardwareMap map) {

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
    public void DrivePower(double angle, double power) {
        this.motors.get("frontLeft").setPower(Math.cos(angle + 3*Math.PI/4));
        this.motors.get("frontRight").setPower(Math.cos(angle + Math.PI/4));
        this.motors.get("backRight").setPower(0 - Math.cos(angle + 3*Math.PI/4));
        this.motors.get("backLeft").setPower(0 - Math.cos(angle + Math.PI/4));
    }

    /***
     *
     * @param angle
     * @param distance
     */
    @Override
    public void DriveDistance(double angle, double distance) {

    }

    /***
     *
     * @param angle
     * @param power
     */
    @Override
    public void RotateAngle(double angle, double power) {

    }

    /***
     *
     * @param direction
     * @param power
     */
    @Override
    public void RotateDirection(RotationDirection direction, double power) {

    }
}