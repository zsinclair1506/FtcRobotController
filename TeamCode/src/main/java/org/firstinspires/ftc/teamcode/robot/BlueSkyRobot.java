package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class BlueSkyRobot extends Robot {

    public BlueSkyRobot(HardwareMap map){
        this.mechanisms.put("lowerConveyor", new Conveyor(map));
        this.mechanisms.put("leftConveyor", new Conveyor(map));
        this.mechanisms.put("rightConveyor", new Conveyor(map));
        this.mechanisms.put("intake", new Intake(map));
        this.mechanisms.put("leftShooter", new Shooter(map));
        this.mechanisms.put("rightShooter", new Shooter(map));
        this.mechanisms.put("wobbleArm", new WobbleArm(map));
        this.driveBase = new XDrive(map);
    }
}
