package org.firstinspires.ftc.teamcode.robot.mapping;

/***
 * A map of the motors on the hubs
 */
public enum MotorMap {

    /***
     * The servo opening and closing the gripper
     */
    GRIPPER_SERVO("gripper"),

    /***
     * The continuous rotation servo for the conveyor
     */
    CONVEYOR_CRSERVO("conveyor"),

    /***
     * The front left drivebase DC motor
     */
    XDRIVE_FRONT_LEFT_DC("frontLeft"),

    /***
     * The front right drivebase DC motor
     */
    XDRIVE_FRONT_RIGHT_DC("frontRight"),

    /***
     * The back right drivebase DC motor
     */
    XDRIVE_BACK_RIGHT_DC("backRight"),

    /***
     * The bark left drivebase DC motor
     */
    XDRIVE_BACK_LEFT_DC("backLeft"),

    /***
     * The servo at the end of the intake to grab the rings
     */
    INTAKE_GRAB_SERVO("intakeGrab"),

    /***
     * The continuous rotation servo for the up and down of the intake mechanism
     */
    INTAKE_UP_CRSERVO("intakeUp"),

    /***
     * The servo for the rotation of the intake mechanism
     */
    INTAKE_ROTATE_SERVO("intakeRotate"),

    /***
     * The motor for the whackystick shooter
     *
     */
    SHOOTER_DC("whackyStick"),

    /***
     * The servo for the loader mechanism
     */
    LOADER_SERVO("loader"),

    /***
     * The continuous rotation servo for the arm mounted to the chassis (shoulder)
     */
    ARM_SHOULDER_CRSERVO(""),

    /***
     * The continuous rotation servo for the middle joint of the arm (elbow)
     */
    ARM_ELBOW_CRSERVO(""),

    /***
     * The continuous rotation servo for the gripper (wrist)
     */
    ARM_WRIST_CRSERVO("");

    /***
     * The string the motor is defined with on the hubs
     */
    private final String motorName;

    /***
     * Constructor assigning the name to the enumeration
     * @param motorName the name of the motor in the phone configuration
     */
    MotorMap(String motorName){
        this.motorName = motorName;
    }

    /***
     * Get the name of the motor in the phone configuration
     * @return the name of the motor from the phone configuration
     */
    public String getMotorName(){
        return this.motorName;
    }
}
