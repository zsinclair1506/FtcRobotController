package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.hardware.DcMotor;

/***
 * Runs a DC motor to a position asynchronously
 */
public class DCToPositionThread extends Thread {
    // As a note, I'm assuming Telemetry is not thread safe so not using it here
    private DcMotor motor;
    private double motorPower;
    private int desiredPosition;

    /***
     * Constructor for the thread
     * @param motor the motor to run asynchronously
     */
    public DCToPositionThread (DcMotor motor, int position){
        this.motor = motor;
        this.desiredPosition = position;
    }

    /***
     * Initialises motor parameters
     * TODO experiment to get this working
     */
    private void init(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(1440); // TODO this or PID control?
    }

    /***
     * Returns whether this thread is running or not
     * @return true if the thread is running
     */
    public boolean isRunning(){
        return this.isAlive();
    }

    /***
     * Runs the DCMotor to the desired position
     */
    @Override
    public void run() {
        this.init();
        try {
            do {
                motor.setPower(motorPower);
                Thread.sleep(100);
            } while (motor.getCurrentPosition() < 1440);
        } catch (InterruptedException e) {
            motor.setPower(0);
        }
        finally {
            motor.setPower(0);
        }
    }

    /***
     * Sets the motor power to power or 1, whichever is smaller
     * @param power the power to set the shooter motor to run at
     */
    public void setPower(double power) {
        this.motorPower = (power > 1 ? 1 : power);
    }
}
