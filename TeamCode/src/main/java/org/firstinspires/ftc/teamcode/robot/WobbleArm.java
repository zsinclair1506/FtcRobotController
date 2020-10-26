package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

/***
 * The class controlling the wobble goal arm mechanism
 */
public class WobbleArm extends Mechanism {
    //private ?? motors
    private Gripper gripper;

    /***
     * Wobblearm constructor
     * @param map hardware map of the robot
     */
    public WobbleArm(HardwareMap map) {
        // assign motors here
        this.gripper = new Gripper(map);
    }

    /***
     * Rotates the arm to a set @angle with a power
     * @param angle the angle to move the arm to [0-1]
     * @param power the power with which to move the arm [0-1]
     */
    public void rotate(double angle, double power) {
        // this will need threading
    }

    /***
     * Rotates the arm in a direction at a power
     * @param direction the rotation direction the arm will rotate in
     *                  (Clockwise or Counter Clockwise)
     * @param power the power to rotate the arm with [0-1]
     */
    public void rotate(RotationDirection direction, double power) {

    }

    /***
     * Opens the gripper on the arm
     */
    public void gripperOpen() {
        this.gripper.openGripper();
    }

    /***
     * Closes the gripper on the arm
     */
    public void gripperClose() {
        this.gripper.closeGripper();
    }

    /***
     * Stores the arm in initial condition
     */
    public void storeArm() {
        rotate(0, 1);
        gripperOpen();
    }

    /***
     * Moves the arm up or down at a certain power.
     * @param direction true for moving the arm up.
     * @param power the power with which to move the arm.
     */
    public void armUpDown(boolean direction, double power) {

    }

    /***
     * Moves the arm up or down.
     * @param direction true for moving the arm up.
     */
    public void armUpDown(boolean direction) {
        armUpDown(direction, 1);
    }

    /***
     * Moves the arm in or out at a certain power.
     * @param direction true for moving the arm in.
     * @param power the power with which to move the arm.
     */
    public void armInOut(boolean direction, double power) {

    }

    /***
     * Moves the arm in or out.
     * @param direction true for moving the arm in.
     */
    public void armInOut(boolean direction) {
        armInOut(direction, 1);
    }

    /***
     * Moves the arm to specific 3D coordinates if possible.
     * @param r the radius from the arm mount point.
     * @param angle the angle above or below the horizontal.
     * @param rotation the rotation of the base, 0 at stored.
     * @throws IllegalArgumentException if the arm cannot achieve the desired input.
     */
    public void moveTo(double r, double angle, double rotation) throws IllegalArgumentException {
        throw new IllegalArgumentException("message");
    }

    /***
     * Tilt the gripper on the arm up.
     */
    public void gripperTiltUp() {
        this.gripper.tiltUp();
    }

    /***
     * Tilt the gripper on the arm down.
     */
    public void gripperTiltDown() {
        this.gripper.tiltDown();
    }


    private void inverseKinematics() {

    }

}