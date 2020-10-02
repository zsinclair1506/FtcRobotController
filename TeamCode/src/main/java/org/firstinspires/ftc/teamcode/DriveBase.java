package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.RotationDirection;

import java.util.HashMap;

public abstract class DriveBase {
    protected HashMap<String, DcMotor> motors;

    public abstract void DrivePower (float angle, float power);
    public abstract void DriveDistance (float angle, float distance);

    public abstract void RotateAngle (float angle, float power);
    public abstract void RotateDirection (RotationDirection direction, float power);

}
