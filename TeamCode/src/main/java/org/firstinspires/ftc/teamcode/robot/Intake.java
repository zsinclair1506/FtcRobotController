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
    private double servoUpTime = 0 ;
    private double servoDownTime = 0;
    private boolean servoLifting = false;
    private boolean servoLowering = false;
    private boolean top = true;

    protected final int DESIRED_UP_TIME_MS = 4200;
    protected final int DESIRED_DOWN_TIME_MS = 3900;
    private final double VERT_UP = -0.6; // Servo Power
    private final double REST_POWER = -0.1; // rest power

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
        this.servoThread = new CRServoThread(vertServo, true);
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

        /***
         * Constructor for the thread
         * @param crservo the servo to run asynchronously
         * @param down the direction of travel, true is down
         *
         */
        public CRServoThread (CRServo crservo, boolean down){
            this.crservo = crservo;
            this.down = down;
        }

        /***
         * Initialises servo parameters
         */
        private void init(){
            this.runTime.reset();
            this.crservo.getController().pwmEnable();
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
            try {
                this.init();

                do{
                    this.crservo.setPower(down ? this.SERVO_POWER : -this.SERVO_POWER);
                    Thread.sleep(100);
                } while
                (this.runTime.milliseconds() < (down ? DESIRED_DOWN_TIME_MS : DESIRED_UP_TIME_MS));

            } catch (InterruptedException e) {
                telemetry.addData("Intake Lift Interrupted. Runtime: ", this.runTime.milliseconds());
            }
            finally{
                this.crservo.setPower(REST_POWER);
                top = !top;
            }
        }

        /***
         * Sets the servo direction for the thread
         * @param down whether it should go down or not
         */
        public void setDirection(boolean down) {
            this.down = down;
        }
    }

    /***
     * Lifts the intake mechanism up from the floor
     */
    public void lift(){
        if(!servoThread.isRunning()) {
            this.vertServo.setPower(VERT_UP);
//            if(!this.servoLifting){
//                if(this.servoLowering){
//                    this.servoDownTime += this.servoRunTime.milliseconds();
//                }
//                this.servoRunTime.reset();
//                this.servoLifting = true;
//                this.servoLowering = false;
//                this.vertServo.getController().pwmEnable();
//                this.vertServo.setPower(VERT_UP);
//            }
        }
    }

    /***
     * Lowers the intake mechanism to the floor
     */
    public void lower(){
        if(!servoThread.isRunning()) {
            this.vertServo.setPower(-VERT_UP);
//            if(this.getPosition() != IntakePosition.DROP_OFF){
//                if(!this.servoLowering){
//                    if(this.servoLifting){
//                        this.servoUpTime += this.servoRunTime.milliseconds();
//                    }
//                    this.servoRunTime.reset();
//                    this.servoLowering = true;
//                    this.servoLifting = false;
//                    this.vertServo.getController().pwmEnable();
//                    this.vertServo.setPower(-VERT_UP);
//                }
//            }
        }
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

    /***
     * Stop all motion on the intake mechanism
     */
    public void stop(){
        if(!servoThread.isRunning()) {
            this.vertServo.setPower(REST_POWER);
//            this.vertServo.getController().pwmDisable();
//            if(servoLifting){
//                this.servoUpTime += this.servoRunTime.milliseconds();
//                this.servoLifting = false;
//            }
//            else if(servoLowering){
//                this.servoDownTime += this.servoRunTime.milliseconds();
//                this.servoLowering = false;
//            }
        }
    }

    public void cancel(){
        if(!servoThread.isRunning()) {
            this.servoThread.interrupt();
        }
    }

    /***
     * Go through the actions to drop a ring off at the hopper
     */
    public void dropOff(){
        if(!this.servoThread.isRunning() && this.getRotation() == IntakePosition.INTAKE) {
            this.servoThread.setDirection(top);
            this.servoThread.run();
        }
    }

    /***
     * Rotates the intake mechanism to a set location
     * @param position the position to rotate the intake to (INTAKE or DROP_OFF)
     */
    private void rotate(IntakePosition position){
        this.rotationServo.setPosition(position.getPosition());
        this.rotation = position;
        this.telemetry.addData("position", position.getPosition());
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
    public IntakePosition getRotation() {
        return this.rotation;
    }
}
