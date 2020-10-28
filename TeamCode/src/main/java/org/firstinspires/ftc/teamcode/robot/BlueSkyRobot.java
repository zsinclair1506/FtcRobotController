package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.lib.RotationDirection;

public class BlueSkyRobot extends Robot {

    public BlueSkyRobot(HardwareMap map){
        this.mechanisms.put("lowerConveyor", new Conveyor(map));
        this.mechanisms.put("leftConveyor", new Conveyor(map));
        this.mechanisms.put("rightConveyor", new Conveyor(map));
        this.mechanisms.put("intake", new Intake(map));
        this.mechanisms.put("leftShooter", new Shooter(map));
        this.mechanisms.put("rightShooter", new Shooter(map));
        this.mechanisms.put("wobbleArm", new WobbleArm(map));
        this.mechanisms.put("switcher", new Switcher(map));
        this.driveBase = new XDrive(map);
    }

    public void runIntake(){
        ((Intake)this.mechanisms.get("intake")).runIntake();
    }

    public void stopIntake(){
        ((Intake)this.mechanisms.get("intake")).stopIntake();
    }

    public void drive(double direction, double power){
        ((XDrive)this.driveBase).drivePower(direction, power);
    }

    public void rotate(RotationDirection direction, double power){
        ((XDrive)this.driveBase).rotateDirection(direction, power);
    }

    public void runEject(){
        ((Intake)this.mechanisms.get("intake")).runEject();
    }

    public void shootLeft(){
        ((Shooter)this.mechanisms.get("leftShooter")).shoot();
    }

    public void shootRight(){
        ((Shooter)this.mechanisms.get("rightShooter")).shoot();
    }

    public void clawClose(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).closeGripper();
    }

    public void clawOpen(){
        ((WobbleArm)this.mechanisms.get("wobbleArm")).openGripper();
    }

    public void switchConveyors(RotationDirection direction){
        ((Switcher)this.mechanisms.get("switcher")).moveRing(direction);
    }
}
