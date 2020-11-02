package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.lib.IntakePosition;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;
import org.firstinspires.ftc.teamcode.robot.lib.Vector;
import org.firstinspires.ftc.teamcode.robot.mapping.MechanismMap;

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
        this.setDriveBase(new XDrive(map, telemetry, this));
        this.addMechanism(MechanismMap.LOADER.getName(), new Loader(map, telemetry, this));
        this.addMechanism(MechanismMap.WOBBLE_GOAL_ARM.getName(),
                new WobbleArm(map, telemetry, this));
        this.addMechanism(MechanismMap.CONVEYOR.getName(),
                new Conveyor(map, telemetry, this));
        this.addMechanism(MechanismMap.INTAKE.getName(),
                new Intake(map, telemetry, this));
        this.addMechanism(MechanismMap.SHOOTER.getName(), new Shooter(map, telemetry, this));
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
        ((Shooter)this.getMechanism(MechanismMap.SHOOTER.getName())).shoot();
    }

    /***
     * TODO figure out how to use this
     * @see Shooter for more information.
     */
    public void shooterFeedMe(){
        ((Shooter)this.getMechanism(MechanismMap.SHOOTER.getName())).feedMe();
    }

    /***
     * @see Gripper for more information.
     */
    public void gripperClose(){
        ((WobbleArm)this.getMechanism(MechanismMap.WOBBLE_GOAL_ARM.getName())).gripperClose();
    }

    /***
     * @see Gripper for more information.
     */
    public void gripperOpen(){
        ((WobbleArm)this.getMechanism(MechanismMap.WOBBLE_GOAL_ARM.getName())).gripperOpen();
    }

    /***
     * @see Conveyor for more information.
     */
    public void conveyorRun(){
        ((Conveyor)this.getMechanism(MechanismMap.CONVEYOR.getName())).convey();
    }

    /***
     * @see WobbleArm for more information.
     */
    public void gripperLower(){
        ((WobbleArm)this.getMechanism(MechanismMap.WOBBLE_GOAL_ARM.getName())).gripperTiltDown();
    }

    /***
     * @see WobbleArm for more information.
     */
    public void gripperRaise(){
        ((WobbleArm)this.getMechanism(MechanismMap.WOBBLE_GOAL_ARM.getName())).gripperTiltUp();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeLower(){
        ((Intake)this.getMechanism(MechanismMap.INTAKE.getName())).lower();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeLift(){
        ((Intake)this.getMechanism(MechanismMap.INTAKE.getName())).lift();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeGrab(){
        ((Intake)this.getMechanism(MechanismMap.INTAKE.getName())).grab();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeRelease(){
        ((Intake)this.getMechanism(MechanismMap.INTAKE.getName())).release();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeCycle(){
        ((Intake)this.getMechanism(MechanismMap.INTAKE.getName())).dropOff();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeRotate(){
        ((Intake) this.getMechanism(MechanismMap.INTAKE.getName())).rotate();
    }

    /***
     * @see Intake for more information.
     */
    public void intakeStop(){
        ((Intake)this.getMechanism(MechanismMap.INTAKE.getName())).stop();
    }

    /***
     * @see Loader for more information.
     */
    public void loaderLoad(){
        ((Loader)this.getMechanism(MechanismMap.LOADER.getName())).load();
    }

    /***
     * @see Loader for more information.
     */
    public void loaderRaise(){
        ((Loader)this.getMechanism(MechanismMap.LOADER.getName())).raise();
    }

    /***
     * @see Loader for more information.
     */
    public void loaderLower(){
        ((Loader)this.getMechanism(MechanismMap.LOADER.getName())).lower();
    }

    /***
     * @see DriveBase for more information.
     */
    public void drive(){
        ((XDrive) this.getDriveBase()).drive();
    }
}
