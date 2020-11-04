package org.firstinspires.ftc.teamcode.robot.lib;

public enum IntakePosition {
    DROP_OFF(0.9),
    INTAKE(0.5),
    ;

    private final double position;

    IntakePosition(double position){
        this.position = position;
    }

    public double getPosition() {
        return this.position;
    }
}
