package org.firstinspires.ftc.teamcode.robot;

/***
 * A map of the motors on the hubs
 */
public enum MotorMap {

    /***
     * The servo opening and closing the gripper
     */
    GRIPPER_SERVO("gripperservo"),

    /***
     * The continuous rotation servo for the conveyor
     */
    CONVEYOR_CRSERVO(""),

    /***
     * The front left drivebase DC motor
     */
    XDRIVE_FRONT_LEFT_DC(""),

    /***
     * The front right drivebase DC motor
     */
    XDRIVE_FRONT_RIGHT_DC(""),

    /***
     * The back right drivebase DC motor
     */
    XDRIVE_BACK_RIGHT_DC(""),

    /***
     * The bark left drivebase DC motor
     */
    XDRIVE_BACK_LEFT_DC(""),

    /***
     * The servo at the end of the intake to grab the rings
     */
    INTAKE_GRAB_SERVO(""),

    /***
     * The continuous rotation servo for the up and down of the intake mechanism
     */
    INTAKE_UP_CRSERVO(""),

    /***
     * The servo for the rotation of the intake mechanism
     */
    INTAKE_ROTATE_SERVO(""),

    /***
     * The motor for the whackystick shooter
     *
     */
    SHOOTER_DC(""),

    /***
     * The servo for the loader mechanism
     */
    LOADER_SERVO(""),

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
    private String motorName;

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