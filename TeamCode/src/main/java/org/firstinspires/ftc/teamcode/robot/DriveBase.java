package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;

import java.util.HashMap;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public abstract class DriveBase {
    protected HashMap<String, DcMotor> motors;


    public DriveBase(HardwareMap map) {

    }

    public abstract void drivePower (double angle, double power);
    public abstract void drivePower (Vector driveVector);
    public abstract void driveDistance (double angle, double distance);

    public abstract void rotateAngle (double angle, double power);
    public abstract void rotateDirection (RotationDirection direction, double power);

    public abstract Vector motorNormalise(Vector vector);

    public double IMUdriveCorrection(double desiredHeading){

        return 0;
    }
}
