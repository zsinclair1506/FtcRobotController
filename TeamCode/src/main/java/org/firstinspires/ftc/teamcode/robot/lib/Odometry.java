package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

import java.util.HashMap;

public class Odometry{
    private boolean initDriveDistance = true;
    private boolean initRotateAngle = true;
    private HashMap<MotorMap, Integer> startPos = new HashMap<>();

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private Telemetry telemetry;
    private double globalAngle;

    /***
     * Constructor for Odometry, initialises
     * @param map
     * @param telemetry
     */
    public Odometry(HardwareMap map, Telemetry telemetry) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        this.imu = map.get(BNO055IMU.class, "imu");

        this.imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.

        this.telemetry = telemetry;
        telemetry.addData("imu calibration status", imu.getCalibrationStatus().toString());
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        this.lastAngles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        this.globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // TODO: check to make sure that the axis selection is correct
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = this.imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - this.lastAngles.firstAngle;

        if (deltaAngle < (-Math.PI))
            deltaAngle += (2 * Math.PI);
        else if (deltaAngle > Math.PI)
            deltaAngle -= (2 * Math.PI);

        this.globalAngle += deltaAngle;

        this.lastAngles = angles;

        return this.globalAngle;
    }

    /**
     * Check the heading, if it isn't straight, calculate required adjustment
     * + adjustment is left
     * - adjustment is left
     *
     * @return Ajustment/ Correction for heading.
     */
    public double checkDirection() {
        // Gain is determined by the sensitivity of adjustment to direction changes
        // Experimenting is required, this has to be changed in the program, rather than by a control
        // Gain is correct when you maintain a straight heading
        double correction, angle, gain = .10;

        angle = this.getAngle();

        if (angle == 0)
            correction = 0;             // Adjustment is not needed as there is no error
        else
            correction = -angle;        // Invert the angle to get the necessary correction

        correction = correction * gain;

        return correction;
    }

    /***
     * Initialises the base encouder counts for a run of the drivebase.
     * @param motors a hashmap of the motors to initialise
     */
    public void driveDistanceInit(HashMap<MotorMap, DcMotor> motors){
        if(this.initDriveDistance){ // first run
            for(DcMotor motor : motors.values()){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            for(MotorMap motorName : motors.keySet()){
                this.startPos.put(motorName, motors.get(motorName).getCurrentPosition());
            }

            this.initDriveDistance = false;
        }
    }

    /***
     * Gets the starting position of the motor encoders
     * @return
     */
    public HashMap<MotorMap, Integer> getStartPos() {
        return this.startPos;
    }

    public void rotateAngleInit() {
        if(this.initRotateAngle){
            this.resetAngle();
            this.initRotateAngle = false;
        }
    }

    public void rotateAngleReset(){
        this.initRotateAngle = true;
    }

    public void driveDistanceReset(){
        this.initDriveDistance = true;
    }
}
