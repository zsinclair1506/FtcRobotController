package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.RotationDirection;


public class XDrive extends DriveBase {


    public XDrive(HardwareMap map) {

        this.motors.put("frontLeft", map.get(DcMotor.class, "frontLeft"));
        this.motors.put("frontRight", map.get(DcMotor.class, "frontRight"));
        this.motors.put("backRight", map.get(DcMotor.class, "backRight"));
        this.motors.put("backLeft", map.get(DcMotor.class, "backLeft"));
    }

    @Override
    public void DrivePower(float angle, float power) {

    }

    @Override
    public void DriveDistance(float angle, float distance) {

    }

    @Override
    public void RotateAngle(float angle, float power) {

    }

    @Override
    public void RotateDirection(RotationDirection direction, float power) {

    }
}