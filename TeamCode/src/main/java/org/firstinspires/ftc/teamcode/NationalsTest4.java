package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Set;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DirectionMethods;
import org.firstinspires.ftc.teamcode.robot.lib.IntakePositions;

import static org.firstinspires.ftc.teamcode.robot.lib.IntakePositions.ONE;
import static org.firstinspires.ftc.teamcode.robot.lib.IntakePositions.THREE;
import static org.firstinspires.ftc.teamcode.robot.lib.IntakePositions.TWO;

/**
 * Backup OpMode using inefficient programming instead of the Object Oriented Prorgamming originally
 * planned for use in the 2020-21 season.
 * @author Zac Sinclair
 */
@TeleOp (name="Old School OpMode", group="Back-Ups")

//TODO: Add in Telemetry to all Methods

public class NationalsTest4 extends LinearOpMode {
    // Declare OpMode members
        // Runtime timer - default mesurement in seconds
    private ElapsedTime runtime;
        // Wheel motor driver hardware variables
    private DcMotor frontLeft, frontRight, backRight, backLeft;
    private CRServo intakeLift;
    private Servo intakeRotate, intakeClaw;

    // constructor
    public NationalsTest4 () {
            // Note - can't initialise DcMotor variables here - must be done inside runOpMode()
            // method below
            // Instantiate a runtime timer
        runtime = new ElapsedTime();
    } // end constructor
        // Initialise hardware - Note that this can't be done in the constructor as
        // JBot Java requires that it be done in the runOpMode() method
    
    // Methods for robot movement control
        // Move robot forwards or backwards

    //TODO: Implement Slow-Mode

    /***
     * The default drive for this instruction set, using the float value from the joystick to
     * give a power for the motors
     * 
     * @param forwardPower, the power used by the motors to drive forward or backward [0-1]
     */
    private void driveforward(float forwardPower){
        do {
            forwardPower = gamepad1.left_stick_y;

            frontLeft.setPower(forwardPower);
            frontRight.setPower(-forwardPower);
            backRight.setPower(-forwardPower);
            backLeft.setPower(forwardPower);
        }while(gamepad1.left_stick_y != 0);
    }

    /***
     * The default left-right drive system for this Class, using the x-axis of the left joystick to
     * give a power for the motors
     *
     * @param strafePower, the power put out the motors to drive the robot left or right [0-1]
     */
    private void driveSide(float strafePower){
        do {
            strafePower = gamepad1.left_stick_x;

            frontLeft.setPower(strafePower);
            frontRight.setPower(strafePower);
            backRight.setPower(-strafePower);
            backLeft.setPower(-strafePower);
        }while(gamepad1.left_stick_x != 0);
    }

    /***
     * The default method for spinning the robot on the spot
     *
     * @param spinPower, the power output by the motors when rotating the robot
     *
     * TODO: Look into using enumerator and buttons for controls
     */
    private void driveSpin(float spinPower){
        do {
            spinPower = gamepad1.right_stick_x;

            frontLeft.setPower(spinPower);
            frontRight.setPower(spinPower);
            backRight.setPower(spinPower);
            backLeft.setPower(spinPower);
        }while(gamepad1.right_stick_x != 0);
    }

    /***
     * Secondary method for robot movement, allows for omni directional movement
     *
     * @param angle
     * @param power
     */
    private void drive(double angle, double power){
        frontLeft.setPower(power * Math.cos(angle + 3*Math.PI/4));
        frontRight.setPower(power * Math.cos(angle + Math.PI/4));
        backRight.setPower(power * (0 - Math.cos(angle + 3*Math.PI/4)));
        backLeft.setPower(power * (0 - Math.cos(angle + Math.PI/4)));
    } //TODO: Test and implement controls, if there is time.


    /***
     *
     */
    private void brake(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /**
     * Mechanism Methods
     */

    private void intakeLift(float liftPower){
        do {
            liftPower = gamepad2.left_stick_y;

            intakeLift.setPower(liftPower);
        }while(gamepad2.left_stick_y != 0);
    }

    private void intakeOpen(){
        if (gamepad2.a){
            intakeClaw.setPosition(0.5);
        }
    }

    private void intakeClose(){
        if(gamepad2.b){
            intakeClaw.setPosition(1.0);
        }
    }

    private void intakeRotate(IntakePositions position) {
        switch (position) {
            case ONE:
                intakeRotate.setPosition(0.0);
                break;
            case TWO:
                intakeRotate.setPosition(0.33);
                break;
            case THREE:
                intakeRotate.setPosition(0.66);
                break;
        }
    }
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        //TODO: Implement new control methods into Direction Methods
        DirectionMethods dMethod = new DirectionMethods(hardwareMap);

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

            // Initialize the hardware variables - ***can only be done inside runOpMode()***.
            // Note that the strings used here as parameters to 'get' must correspond to
            // the names assigned during the robot configuration step
            // (using the FTC Robot Controller app on the phone).
        
            // Wait for the game to start (driver presses PLAY)          
        waitForStart();
            // Start timer - saves running time in seconds
        runtime.reset();
   
            // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Drive controls
            if (gamepad1.left_stick_y != 0) {
                driveforward(gamepad1.left_stick_y);
            }else if (gamepad1.left_stick_x != 0){
                driveSide(gamepad1.left_stick_x);
            }else if (gamepad1.right_stick_x != 0){
                driveSpin(gamepad1.right_stick_x);
            }else if (gamepad2.dpad_right){
                intakeRotate(THREE);
            }else if (gamepad2.dpad_up) {
                intakeRotate(TWO);
            }else if (gamepad2.dpad_left) {
                intakeRotate(ONE);
            }else if (gamepad2.a){
                intakeOpen();
            }else if (gamepad2.b){
                intakeClose();
            }
            else {
                brake();
            }

                // Show the elapsed game time and wheel power.
            telemetry.addData("Status: ", "/nRun Time: " + runtime.toString() + "secs");
            telemetry.addData("Motors", "left (%.2f), right (%.2f), rleft(%.2f), rright(%.2f)");
            telemetry.update();
        } // end while (opModeIsActive()) Loop
    } // end runOpMode()
} // end class NationalsTest4
