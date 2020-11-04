package org.firstinspires.ftc.teamcode.robot.mapping;

public enum InterlockMap {
    ;

    public enum Actions {
        LOADER_LOWER,
        LOADER_RAISE,
        INTAKE_ROTATE,
        INTAKE_LOWER,
        CONVEYOR_RUN,
        ;
    }

    public enum Blockers {
        SHOOTER_SHOOTING,
        LOADER_RAISED,
        INTAKE_LOWERED,
        INTAKE_ROTATED_IN,
        ;
    }
}
