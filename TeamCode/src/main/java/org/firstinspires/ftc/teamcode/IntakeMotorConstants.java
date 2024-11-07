package org.firstinspires.ftc.teamcode;

public final class IntakeMotorConstants {

    static final double INTAKE_MOTOR_REVOLUTION_TIME = 25;
    static final double INTAKE_MOTOR_POWER = 0.4;
    static final double INTAKE_MOTOR_EJECT_POWER = -0.6;

    enum IntakeDirection {
        INTAKE,
        EJECT,
        IDLE
    }

    private IntakeMotorConstants() {}
}
