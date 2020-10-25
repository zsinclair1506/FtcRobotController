package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.IntakePosition;
import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

public class BlueSkyRobot extends Robot {

    public BlueSkyRobot(HardwareMap map){
        this.mechanisms.put("Conveyor", new Conveyor(map));
        this.mechanisms.put("intake", new Intake(map));
        this.mechanisms.put("shooter", new Shooter(map));
        this.mechanisms.put("wobbleArm", new WobbleArm(map));
        this.driveBase = new XDrive(map);
    }

    public void drive(double direction, double power){
        ((XDrive)this.driveBase).drivePower(direction, power);
    }

    public void rotate(RotationDirection direction, double power){
        ((XDrive)this.driveBase).rotateDirection(direction, power);
    }

    public void shooterShoot(){
        ((Shooter)this.mechanisms.get("shooter")).shoot();
    }

    public void shooterFeedMe(){
        ((Shooter)this.mechanisms.get("shooter")).feedMe();
    }

    public void clawClose(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperClose();
    }

    public void clawOpen(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperOpen();
    }

    public void conveyorRun(){
        ((Conveyor)this.mechanisms.get("conveyor")).convey();
    }

    public void gripperLower(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperTiltDown();
    }

    public void gripperRaise(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).gripperTiltUp();
    }

    public void intakeLower(){
        ((Intake)this.mechanisms.get("intake")).lower();
    }

    public void intakeLift(){
        ((Intake)this.mechanisms.get("intake")).lift();
    }

    public void intakeLRotate(IntakePosition position){
        ((Intake)this.mechanisms.get("intake")).rotate(position);
    }

    public void intakeGrab(){
        ((Intake)this.mechanisms.get("intake")).grab();
    }

    public void intakeRelease(){
        ((Intake)this.mechanisms.get("intake")).release();
    }

    public void intakeDropOff(){
        ((Intake)this.mechanisms.get("intake")).dropOff();
    }

    public void intakeStop(){
        ((Intake)this.mechanisms.get("intake")).stop();
    }

    public void wobbleArmRotate(RotationDirection direction){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).rotate(direction, 1);
    }

    public void wobbleArmMove(double angle, double power){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).moveArm(angle, power);
    }
}
