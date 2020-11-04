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
    private CRServoThread servoThread = new CRServoThread(vertServo, VERT_UP);
    private IntakePosition position = IntakePosition.INTAKE;

    protected static int DESIRED_RUN_TIME_MS = 1000;
    private static double VERT_UP = 0.8;
    private static double GRAB_POSITION = 0.5;
    private static double CLOSED_POSITION = 0.2;

    /***
     * Class to handle running a continuous servo asynchronously. For moving the intake to the top
     * or bottom of the rail by running it for a certain amount of time. Since CRServo does not have
     * a position sensor or control, it will have to be time based. (This may be affected by the
     * battery level of the robot during operation.
     */
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
            this.runTume.reset();
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

            do{
                this.crservo.setPower(this.servoPower);
            } while (this.runTume.milliseconds() < DESIRED_RUN_TIME_MS);

            this.crservo.setPower(0);
        }

        /***
         * Sets the servo power to inpower or 1, whichever is smaller
         * @param inPower the power to set the shooter motor to run at
         */
        public void setPower(double inPower) {
            int sign = inPower < 1 ? -1 : 1;
            this.servoPower = sign * (Math.abs(inPower) > 1 ? 1 : Math.abs(inPower));
        }
    }

    /***
     * Intake constructor
     * @param map the hardware map of the robot
     */
    public Intake(HardwareMap map, Telemetry telemetry, Robot robot) {
        super(telemetry, robot);
        this.rotationServo = map.get(Servo.class, MotorMap.INTAKE_ROTATE_SERVO.getMotorName());
        this.vertServo = map.get(CRServo.class, MotorMap.INTAKE_UP_CRSERVO.getMotorName());
        this.grabServo = map.get(Servo.class, MotorMap.INTAKE_GRAB_SERVO.getMotorName());
    }

    /***
     * Lifts the intake mechanism up from the floor
     */
    public void lift(){
        this.vertServo.getController().pwmEnable();
        this.vertServo.setPower(VERT_UP);
    }

    /***
     * Causes the intake mechanism to grab a ring
     */
    public void grab(){
        this.grabServo.setPosition(GRAB_POSITION);
    }

    /***
     * Causes the intake mechanism to release a ring
     */
    public void release(){
        this.grabServo.setPosition(CLOSED_POSITION);
    }

    /***
     * Lowers the intake mechanism to the floor
     */
    public void lower(){
        this.vertServo.getController().pwmEnable();
        this.vertServo.setPower(-VERT_UP);
    }

    /***
     * Stop all motion on the intake mechanism
     */
    public void stop(){
        this.servoThread.interrupt();
        this.vertServo.getController().pwmDisable();
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
    private void rotate(IntakePosition position){
        this.rotationServo.setPosition(position.getPosition());
        this.position = position;
    }

    /***
     * Rotate to the other position for the intake. (INTAKE or DROP_OFF)
     */
    public void rotate(){
        if (this.position == IntakePosition.INTAKE){
            this.rotate(IntakePosition.DROP_OFF);
        }
        else{
            this.rotate(IntakePosition.INTAKE);
        }
    }

    /***
     * Get the current position of the intake.
     * @return the current position of the intake
     */
    public IntakePosition getPosition() {
        return this.position;
    }
}
