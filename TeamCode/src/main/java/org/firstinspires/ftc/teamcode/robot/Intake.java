package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.IntakePosition;
import org.firstinspires.ftc.teamcode.robot.mapping.MotorMap;

/***
 * This class is for the intake mechanism on the robot. This mechanism has linear motion bottom
 * to top. Has a pop out and pop in action for gripping the ring. It also has rotation to place
 * ring in the passive hopper.
 */
public class Intake extends Mechanism {
    private Servo rotationServo;
    private CRServo vertServo;
    private Servo grabServo;

    private class CRServoThread extends Thread {
        // As a note, I'm assuming Telemetry is not thread safe so not using it here
        private CRServo crservo;
        private double servoPower;
        private ElapsedTime runTume = new ElapsedTime();

        /***
         * Constructor for the thread
         * @param crservo the servo to run asynchronously
         */
        public CRServoThread (CRServo crservo, double power){
            this.crservo = crservo;
            this.servoPower = power;
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

        }

        /***
         * Sets the servo power to inpower or 1, whichever is smaller
         * @param inPower the power to set the shooter motor to run at
         */
        public void setPower(double inPower) {
            this.servoPower = (inPower > 1 ? 1 : inPower);
        }
    }

    /***
     * Intake constructor
     * @param map the hardware map of the robot
     */
    public Intake(HardwareMap map, Telemetry telemetry){
        super(telemetry);
        this.rotationServo = map.get(Servo.class, MotorMap.INTAKE_ROTATE_SERVO.getMotorName());
        this.vertServo = map.get(CRServo.class, MotorMap.INTAKE_UP_CRSERVO.getMotorName());
        this.grabServo = map.get(Servo.class, MotorMap.INTAKE_GRAB_SERVO.getMotorName());
    }

    /***
     * Lifts the intake mechanism up from the floor
     */
    public void lift(){

    }

    /***
     * Causes the intake mechanism to grab a ring
     */
    public void grab(){

    }

    /***
     * Causes the intake mechanism to release a ring
     */
    public void release(){

    }

    /***
     * Lowers the intake mechanism to the floor
     */
    public void lower(){

    }

    /***
     * Stop all motion on the intake mechanism
     */
    public void stop(){

    }

    /***
     * Go through the actions to drop a ring off at the hopper
     */
    public void dropOff(){
        /*
        Will likely have to be a thread since this series of actions will take time and we want
        to continue to move around and do other things. Will have to check for thread running in
        each of the other methods to make sure we're not accessing data concurrently.
         */
    }

    /***
     * Rotates the intake mechanism to a set location
     * @param position the position to rotate the intake to (INTAKE or DROP_OFF)
     */
    public void rotate(IntakePosition position){

    }
}
