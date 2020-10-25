package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

import java.util.HashMap;

public abstract class DriveBase {
    protected HashMap<String, DcMotor> motors;

    public abstract void drivePower(double angle, double power);
    public abstract void driveDistance(double angle, double distance);

    public abstract void rotateAngle(double angle, double power);
    public abstract void rotateDirection(RotationDirection direction, double power);

}
