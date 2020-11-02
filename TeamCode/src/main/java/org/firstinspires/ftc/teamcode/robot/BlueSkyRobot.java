package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.IntakePosition;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

/***
 * The specific robot for this year. Mainly used to pass actions between the Operation Mode and
 * the mechanisms.
 */
public class BlueSkyRobot extends Robot {

    /***
     * Constructor for this years' robot. Creates all the mechanisms and the drivebase.
     * @param map the hardware map containing all the hardware on the robot.
     */
    public BlueSkyRobot(HardwareMap map){
        this.mechanisms.put("conveyor", new Conveyor(map));
        this.mechanisms.put("intake", new Intake(map));
        this.mechanisms.put("shooter", new Shooter(map));
        this.mechanisms.put("wobbleArm", new WobbleArm(map));
        this.driveBase = new XDrive(map);
    }

    /***
     * @see DriveBase for more information.
     * @param direction @see DriveBase for more information.
     * @param power @see DriveBase for more information.
     */
    public void drive(double direction, double power){
        ((XDrive)this.driveBase).drivePower(direction, power);
    }

    /***
     * @see DriveBase for more information.
     * @param direction @see DriveBase for more information.
     * @param power @see DriveBase for more information.
     */
    public void rotate(RotationDirection direction, double power){
        ((XDrive)this.driveBase).rotateDirection(direction, power);
    }

    /***
     * @see Shooter for more information.
     */
    public void shooterShoot(){
        ((Shooter)this.mechanisms.get("shooter")).shoot();
    }

    /***
     * @see Shooter for more information.
     */
    public void shooterFeedMe(){
        ((Shooter)this.mechanisms.get("shooter")).feedMe();
    }

    /***
     * @see Gripper for more information.
     */
    public void clawClose(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperClose();
    }

    /***
     * @see Gripper for more information.
     */
    public void clawOpen(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperOpen();
    }

    /***
     * @see Conveyor for more information.
     */
    public void conveyorRun(){
        ((Conveyor)this.mechanisms.get("conveyor")).convey();
    }

    /***
     * @see WobbleArm for more information.
     */
    public void gripperLower(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperTiltDown();
    }

    /***
     * @see WobbleArm for more information.
     */
    public void gripperRaise(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperTiltUp();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeLower(){
        ((Intake)this.mechanisms.get("intake")).lower();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeLift(){
        ((Intake)this.mechanisms.get("intake")).lift();
    }

    /***
     * @see Intake for more information.
     * @param position @see Intake for more information.
     */
    public void intakeLRotate(IntakePosition position){
        ((Intake)this.mechanisms.get("intake")).rotate(position);
    }

    /***
     * @see Intake for more information.
     */
    public void intakeGrab(){
        ((Intake)this.mechanisms.get("intake")).grab();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeRelease(){
        ((Intake)this.mechanisms.get("intake")).release();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeDropOff(){
        ((Intake)this.mechanisms.get("intake")).dropOff();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeStop(){
        ((Intake)this.mechanisms.get("intake")).stop();
    }
}
