package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.IntakePosition;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;

/***
 * The specific robot for this year. Mainly used to pass actions between the Operation Mode and
 * the mechanisms.
 */
public class BlueSkyRobot extends Robot {

    /***
     * Constructor for this years' robot. Creates all the mechanisms and the drivebase.
     * @param map the hardware map containing all the hardware on the robot.
     */
    public BlueSkyRobot(HardwareMap map, Telemetry telemetry){
        this.addMechanism("conveyor", new Conveyor(map, telemetry));
//        this.addMechanism("intake", new Intake(map, telemetry));
//        this.addMechanism("shooter", new Shooter(map, telemetry));
//        this.addMechanism("wobbleArm", new WobbleArm(map, telemetry));
//        this.addMechanism("loader", new Loader(map, telemetry));
        this.setDriveBase(new XDrive(map, telemetry));
    }

    /***
     * @see DriveBase for more information.
     * @param direction @see DriveBase for more information.
     * @param power @see DriveBase for more information.
     */
    public void setStrafe(double direction, double power){
        ((XDrive)this.getDriveBase()).setStrafe(direction, power);
    }

    /***
     * @see DriveBase for more information.
     * @param driveVector @see DriveBase for more information.
     */
    public void setStrafe(Vector driveVector){
        ((XDrive)this.getDriveBase()).setStrafe(driveVector);
    }

    /***
     * @see DriveBase for more information.
     * @param direction @see DriveBase for more information.
     * @param power @see DriveBase for more information.
     */
    public void setRotate(RotationDirection direction, double power){
        ((XDrive)this.getDriveBase()).setRotation(direction, power);
    }

    public void setRotate(Vector vector){
        ((XDrive)this.getDriveBase()).setRotation(vector);
    }

    /***
     * @see Shooter for more information.
     */
    public void shooterShoot(){
        ((Shooter)this.getMechanism("shooter")).shoot();
    }

    /***
     * TODO figure out how to use this
     * @see Shooter for more information.
     */
    public void shooterFeedMe(){
        ((Shooter)this.getMechanism("shooter")).feedMe();
    }

    /***
     * @see Gripper for more information.
     */
    public void gripperClose(){
        ((WobbleArm)this.getMechanism("wobbleArm")).gripperClose();
    }

    /***
     * @see Gripper for more information.
     */
    public void gripperOpen(){
        ((WobbleArm)this.getMechanism("wobbleArm")).gripperOpen();
    }

    /***
     * @see Conveyor for more information.
     */
    public void conveyorRun(){
        ((Conveyor)this.getMechanism("conveyor")).convey();
    }

    /***
     * @see WobbleArm for more information.
     */
    public void gripperLower(){
        ((WobbleArm)this.getMechanism("wobbleArm")).gripperTiltDown();
    }

    /***
     * @see WobbleArm for more information.
     */
    public void gripperRaise(){
        ((WobbleArm)this.getMechanism("wobbleArm")).gripperTiltUp();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeLower(){
        ((Intake)this.getMechanism("intake")).lower();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeLift(){
        ((Intake)this.getMechanism("intake")).lift();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeGrab(){
        ((Intake)this.getMechanism("intake")).grab();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeRelease(){
        ((Intake)this.getMechanism("intake")).release();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeCycle(){
        ((Intake)this.getMechanism("intake")).dropOff();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeRotate(){
        ((Intake) this.getMechanism("intake")).rotate();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeStop(){
        ((Intake)this.getMechanism("intake")).stop();
    }

    /***
     * @see Loader for more information.
     */
    public void loaderLoad(){
        ((Loader)this.getMechanism("loader")).load();
    }

    /***
     * @see Loader for more information.
     */
    public void loaderRaise(){
        ((Loader)this.getMechanism("loader")).raise();
    }

    /***
     * @see Loader for more information.
     */
    public void loaderLower(){
        ((Loader)this.getMechanism("loader")).lower();
    }

    /***
     * @see DriveBase for more information.
     */
    public void drive(){
        ((XDrive) this.getDriveBase()).drive();
    }
}
