package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private CRServoThread servoThread;
    private IntakePosition rotation = IntakePosition.INTAKE;
    private ElapsedTime servoRunTime = new ElapsedTime();
    private double servoDownTime = 0;
    private boolean isLowering = false;
    private boolean isLifting = false;
    private boolean top = true;

//    protected final int DESIRED_UP_TIME_MS = 4200; // 1.0769 downtime
    protected final int DESIRED_DOWN_TIME_MS = 3900; //
    private final double VERT_UP_POWER = -0.6; // Servo Power
    private final double REST_POWER = -0.1; // rest power
    private final double SERVO_UP_RATIO = 1.0769;

    /***
     * Positions of the intake. Not mutually exclusive.
     */
    public enum IntakePosition {
        DROP_OFF(0.1),
        INTAKE(0.6),
        GRAB_POSITION(0.1),
        CLOSED_POSITION(0.25),
        ;

        private final double position;

        IntakePosition(double position){
            this.position = position;
        }

        public double getPosition() {
            return this.position;
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
        this.servoThread = new CRServoThread(vertServo);
    }

    /***
     * Class to handle running a continuous servo asynchronously. For moving the intake to the top
     * or bottom of the rail by running it for a certain amount of time. Since CRServo does not have
     * a position sensor or control, it will have to be time based. (This may be affected by the
     * battery level of the robot during operation.
     */
    private class CRServoThread extends Thread {
        // As a note, I'm assuming Telemetry is not thread safe so not using it here
        private CRServo crservo;
        private final double SERVO_POWER = 1;
        private ElapsedTime runTime = new ElapsedTime();
        private boolean down = true;
        private double desiredRunTime = 0;
        private boolean fullCycle = false;

        /***
         * Constructor for the thread
         * @param crservo the servo to run asynchronously
         */
        public CRServoThread (CRServo crservo){
            this.crservo = crservo;
        }

        /***
         * Initialises servo parameters
         */
        private void init(){
            this.runTime.reset();
        }

        /***
         * Returns whether this thread is running or not
         * @return true if the thread is running
         */
        public boolean isRunning(){
            return this.isAlive();
        }

        /***
         * Runs the CRServo for the set amount of time in the set direction. Please set time and
         * direction before running.
         */
        @Override
        public void run() {
            try {
                this.init();

                do{
                    this.crservo.setPower(down ? this.SERVO_POWER : -this.SERVO_POWER);
                    Thread.sleep(20);
                } while
                (this.runTime.milliseconds() < this.desiredRunTime);

                if(fullCycle){
                    countTime();

                    isLowering = false;
                    isLifting = false;

                    rotate();
                    Thread.sleep(250);
                    release();
                    Thread.sleep(100);
                    rotate();
                }
            } catch (InterruptedException e) {
                this.interrupt();
                telemetry.addData("Intake Lift Interrupted. Runtime: ", this.runTime.milliseconds());
            }
            finally{
                this.crservo.setPower(REST_POWER);

                if(fullCycle){
                    this.setFullCycle(false);
                    servoDownTime = 0;
                }
                else{
                    if(this.down) {
                        servoDownTime += this.runTime.milliseconds();
                    }
                    else{
                        servoDownTime -= this.runTime.milliseconds();
                    }
                }
            }
        }

        /***
         * Sets the servo direction for the thread
         * @param down whether it should go down or not
         */
        protected void setGoDown(boolean down) {
            this.down = down;
        }

        /***
         * Set the runtime for the thread.
         * @param time the time to run the thread for
         */
        protected void setRunTime(double time)
        {
            this.desiredRunTime = time == 0 ? DESIRED_DOWN_TIME_MS : time;
        }

        /***
         *
         */
        @Override
        public void interrupt(){
            if(this.down) {
                servoDownTime += this.runTime.milliseconds();
            }
            else{
                servoDownTime -= this.runTime.milliseconds();
            }

            this.crservo.setPower(REST_POWER);
        }


        /***
         * Sets whether the thread should perform a full cycle.
         * @param fullCycle whether this thread should perform a full cycle
         */
        public void setFullCycle(boolean fullCycle) {
            this.fullCycle = fullCycle;
        }
    }

    /***
     * Lifts the intake mechanism up from the floor automatically
     */
    public void lift(){
        if(!servoThread.isRunning()) {
            this.countTime();

            if(this.servoDownTime > 0){
                this.servoThread.setRunTime(SERVO_UP_RATIO * (this.servoDownTime));
                this.servoThread.setGoDown(false);
                this.servoThread.start();

                this.isLifting = true;
                this.isLowering = false;
            }
            else{
                this.vertServo.setPower(VERT_UP_POWER);
            }
        }
    }

    /***
     * Lowers the intake mechanism to the floor automatically
     */
    public void lower(){
        if(!servoThread.isRunning()) {
            this.countTime();

            if(this.servoDownTime < this.DESIRED_DOWN_TIME_MS){
                this.servoThread.setRunTime(this.DESIRED_DOWN_TIME_MS - this.servoDownTime);
                this.servoThread.setGoDown(true);
                this.servoThread.start();

                this.isLifting = false;
                this.isLowering = true;
            }
            else{
                this.vertServo.setPower(-VERT_UP_POWER);
            }
        }
    }

    /***
     * Stop all motion on the intake mechanism
     */
    public void stop(){
        if(!servoThread.isRunning()) {
            this.countTime();

            this.vertServo.setPower(REST_POWER);

            this.isLifting = false;
            this.isLowering = false;
        }
    }

    /***
     * Counts the elapsed time for running
     */
    private void countTime() {
        if(this.isLowering){
            this.servoDownTime += this.servoRunTime.milliseconds();
        }
        else if(this.isLifting){
            this.servoDownTime -= this.servoRunTime.milliseconds() / this.SERVO_UP_RATIO;
        }

        this.servoRunTime.reset();
    }

    /***
     * Causes the intake mechanism to grab a ring
     */
    public void grab(){
        this.grabServo.setPosition(IntakePosition.GRAB_POSITION.getPosition());
    }

    /***
     * Causes the intake mechanism to release a ring
     */
    public void release(){
        this.grabServo.setPosition(IntakePosition.CLOSED_POSITION.getPosition());
    }

    public void cancel(){
        if(!servoThread.isRunning()) {
            this.servoThread.interrupt();
        }
    }

    /***
     * Go through the actions to drop a ring off at the hopper
     */
    public void cycle(){
        if(!this.servoThread.isRunning()
                && this.getRotation() == IntakePosition.INTAKE
                && this.servoDownTime > 0) {
            this.servoThread.setFullCycle(true);
            this.servoThread.setRunTime(SERVO_UP_RATIO * (this.servoDownTime));
            this.servoThread.setGoDown(false);
            this.servoThread.start();
        }
    }

    /***
     * Rotates the intake mechanism to a set location
     * @param position the position to rotate the intake to (INTAKE or DROP_OFF)
     */
    private void rotate(IntakePosition position){
        if(!servoThread.isRunning()) {
            this.rotationServo.setPosition(position.getPosition());
            this.rotation = position;
        }
    }

    /***
     * Rotate to the other position for the intake. (INTAKE or DROP_OFF)
     */
    public void rotate(){
        if(!servoThread.isRunning()){
            if (this.rotation == IntakePosition.INTAKE){
                this.rotate(IntakePosition.DROP_OFF);
            }
            else{
                this.rotate(IntakePosition.INTAKE);
            }
        }
    }

    /***
     * Get the current position of the intake.
     * @return the current position of the intake
     */
    private IntakePosition getRotation() {
        return this.rotation;
    }
}
