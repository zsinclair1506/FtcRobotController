package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

import java.util.HashMap;

public abstract class DriveBase {
    protected HashMap<String, DcMotor> motors;

    public abstract void DrivePower (double angle, double power);
    public abstract void DriveDistance (double angle, double distance);

    public abstract void RotateAngle (double angle, double power);
    public abstract void RotateDirection (RotationDirection direction, double power);

}
