package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUWrapper {

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Telemetry telemetry;
    double globalAngle, power = .30, correction;

    public IMUWrapper(HardwareMap map, Telemetry telemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        this.telemetry = telemetry;

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = map.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        telemetry.addData("imu calibration status", imu.getCalibrationStatus().toString());
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < (-Math.PI))
            deltaAngle += (2 * Math.PI);
        else if (deltaAngle > Math.PI)
            deltaAngle -= (2 * Math.PI);

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Check the heading, if it isn't straight, calculate required adjustment
     * + adjustment is left
     * - adjustment is left
     *
     * @return Ajustment/ Correction for heading.
     */
    private double checkDirection() {
        // Gain is determined by the sensitivity of adjustment to direction changes
        // Experimenting is required, this has to be changed in the program, rather than by a control
        // Gain is correct when you maintain a straight heading
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // Adjustment is not needed as there is no error
        else
            correction = -angle;        // Invert the angle to get the necessary correction

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double frontLeftPower, frontRightPower, rearLeftPower, rearRightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        //TODO: Update to match outputs for XDrive?
        if (degrees < 0) {   // turn right.
            frontLeftPower = -power;
            frontRightPower = -power;
            rearLeftPower = -power;
            rearRightPower = -power;
        } else if (degrees > 0) {   // turn left.
            frontLeftPower = power;
            frontRightPower = power;
            rearLeftPower = power;
            rearRightPower = power;
        } else return;


        /**
         // rotate until turn is completed.
         if (degrees < 0) {
         // On right turn we have to get off zero first.
         while (opModeIsActive() && getAngle() == 0) {
         }

         while (opModeIsActive() && getAngle() > degrees) {
         }
         } else    // left turn.
         while (opModeIsActive() && getAngle() < degrees) {
         }
         */
    }
}
