package org.firstinspires.ftc.teamcode.robot.lib;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoThread extends Thread {
    // As a note, I'm assuming Telemetry is not thread safe so not using it here
    private Servo servo;
    private double servoPower;
    private double desiredPosition;

    /***
     * Constructor for the thread
     * @param servo the servo to run asynchronously
     */
    public ServoThread (Servo servo, double position){
        this.servo = servo;
        this.desiredPosition = position;
    }

    /***
     * Initialises servo parameters
     */
    private void init(){
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
                servo.setPosition(desiredPosition);
                Thread.sleep(100);
            } while (servo.getPosition() != desiredPosition);
        } catch (InterruptedException e) {
            servo.setPosition(servo.getPosition());
        }
    }

    /***
     * Sets the motor power to power or 1, whichever is smaller
     * @param power the power to set the shooter motor to run at
     */
    public void setPower(double power) {
        this.servoPower = (power > 1 ? 1 : power);
    }
}
