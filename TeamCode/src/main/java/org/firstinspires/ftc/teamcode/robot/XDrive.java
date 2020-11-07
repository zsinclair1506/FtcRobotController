package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

import java.util.HashMap;


public class XDrive extends DriveBase {
    private boolean init = true;
    private HashMap<String, Integer> startPos = new HashMap<>();

    /***
     *
     * @param map the hardware map of the robot/phone/expansion hub.
     */
    public XDrive (HardwareMap map, Telemetry telemetry, Robot robot) {
        super(telemetry, robot);
        this.addMotor(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName()));
        this.addMotor(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName()));
        this.addMotor(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName()));
        this.addMotor(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName(),
                map.get(DcMotor.class, MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName()));
    }

    /***
     * Combines the power of the rotation and strafe and scales the maximum value to be between 0
     * and 1.
     */
    @Override
    public void drive() {
        this.byOurPowersCombined();
        this.motorNormalise();
//
        for(String motorName : this.getMotors().keySet()){
            this.getMotor(motorName).setPower(this.getDrivePowers().get(motorName));
        }
    }

    /***
     * Drive the robot at an angle at a power.
     * @param angle the angle to drive the robot in, 0 is forward. This base slides
     * @param power the power with which to move the robot
     */
    @Override
    public void setStrafe(double angle, double power) {
        this.setStrafeMotorPower(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName(),
                (-power * Math.cos(angle - Math.PI/4)));
        this.setStrafeMotorPower(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName(),
                (-power * Math.cos(angle + Math.PI/4)));
        this.setStrafeMotorPower(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName(),
                (power * Math.cos(angle - Math.PI/4)));
        this.setStrafeMotorPower(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName(),
                (power * Math.cos(angle + Math.PI/4)));
    }

    /***
     * Drive the robot along a set vector.
     * @param driveVector the vector to drive the robot along
     */
    @Override
    public void setStrafe(Vector driveVector) {
        this.setStrafe(driveVector.getMagnitude(), driveVector.getAngleBetween(Vector.X_2));
    }

    /***
     * Drive the robot a set distance at a certain angle. (Meant to be run in a loop)
     * @param angle the angle to drive the robot in, 0 is forward. This base slides
     * @param distance the distance to drive from current location
     */
    @Override
    public void driveDistance(double angle, double distance) {
        if(init){ // first run
            int i = 0;

            for(String motorName : this.getMotors().keySet()){
                startPos.put(motorName, this.getMotor(motorName).getCurrentPosition());
            }

            
            /**
             * DriveDistance; with reset displacements
             * For each motor in this.getmotor.getvalues.keys
             * for each key in motor map, getting names
             * with name get motor and encoder count
             *
             * with encoder count, new hashmap
             * setting up 4 vectors based on their name and orientation
             * loop to calc vector (Vector sum = new vector 0,0)
             * add 3 vectors and get travel vector
             *
             * compare with desired vector (mag, angle)
             * calculate error in magnitude (displacement of robot)
             * (possible clever magic with circumference, encoder counts and total counts?)
             *
             * magnitude: Distance travelled
             * resultant vector: rv
             * desired vector: dv
             * remaining vector: remv
             *
             * diff between dv and rv gives remv and correction angle needed
             *
             * feed values into setstrafe method
             *
             * call a 0 setrotate
             *
             * if trying to call a hashmap that doesnt exist it returns null
             *
             * null check in byourpowerscombined?
             * if null add 0
             *
             * if checkrotate is needed, use IMU methods
             * setrotate in XDrive, not IMU
             * XDrive will contain the same code as IMU one
             *
             * Copy IMU one to XDrive
             *
             * CALL .DRIVE BECUASE YES
             *
             * if strafe, rotate and other things are null, it will not cause issue bc they will
             * become 0
             *
             * Can all be apart of drivedistance becuase yes
             */
            init = false;
        }
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
        this.setRotateMotorPower(MotorMap.XDRIVE_FRONT_LEFT_DC.getMotorName(), power);
        this.setRotateMotorPower(MotorMap.XDRIVE_FRONT_RIGHT_DC.getMotorName(), power);
        this.setRotateMotorPower(MotorMap.XDRIVE_BACK_RIGHT_DC.getMotorName(), power);
        this.setRotateMotorPower(MotorMap.XDRIVE_BACK_LEFT_DC.getMotorName(), power);
    }

    /***
     * Rotate the robot according to a vector (meant to be run in a loop)
     * @param vector the vector to rotate the drivebase
     */
    @Override
    public void setRotation(Vector vector) {
        RotationDirection direction = Math.cos(vector.getAngleBetween(Vector.X_2)) < Math.PI/2
                ? RotationDirection.CLOCKWISE : RotationDirection.COUNTER_CLOCKWISE ;
        this.setRotation(direction, vector.getMagnitude());
    }

    /***
     * Normalises the motor powers with the maximum value of 1 when input to the motors while
     * preserving the desired ratios of power.
     */
    @Override
    protected void motorNormalise() {
        double maxValue = 0.1;

        // calculate the max value
        for(String motor : this.getMotors().keySet()){
            maxValue = Math.max(maxValue, Math.abs(this.getDrivePowers().get(motor)));
        }

        // only scale the value if it is above 1. This will be most of the time.
        if(maxValue > 1) {
            for (String motor : this.getMotors().keySet()) {
                this.setDrivePower(motor, this.getDrivePowers().get(motor) / maxValue);
            }
        }
    }

    public double getDistance(){

        return 0;
    }

    public void driveDistance(Vector driveVector) {
        this.setStrafe(driveVector.getMagnitude(), driveVector.getAngleBetween(Vector.X_2));
    }
}
