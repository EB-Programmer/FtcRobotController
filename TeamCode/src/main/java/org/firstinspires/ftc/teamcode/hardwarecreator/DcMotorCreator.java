package org.firstinspires.ftc.teamcode.hardwarecreator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotorCreator {

    private final DcMotor motor;

    public static class Builder {

        private final String deviceName;
        private final HardwareMap hardwareMap;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        public Builder(String deviceName, HardwareMap hardwareMap) {
            this.deviceName = deviceName;
            this.hardwareMap = hardwareMap;
        }

        public Builder RunMode(DcMotor.RunMode mode) {
            this.mode = mode;
            return this;
        }

        public Builder Direction(DcMotorSimple.Direction direction) {
            this.direction = direction;
            return this;
        }

        public Builder ZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            this.zeroPowerBehavior = zeroPowerBehavior;
            return this;
        }

        public DcMotorCreator build() {
            return new DcMotorCreator(this);
        }

    }

    private DcMotorCreator (Builder builder) {
        motor = builder.hardwareMap.get(DcMotor.class, builder.deviceName);
        motor.setMode(builder.mode);
        motor.setDirection(builder.direction);
        motor.setZeroPowerBehavior(builder.zeroPowerBehavior);
    }

    public DcMotor getDcMotor() {
        return motor;
    }

}
