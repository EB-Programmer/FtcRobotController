package org.firstinspires.ftc.teamcode.hardwarecreator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoCreator {

    private final Servo servo;

    public static class Builder {
            private final String deviceName;
            private final HardwareMap hardwareMap;
            private Servo.Direction direction;

            public Builder(HardwareMap hardwareMap, String deviceName) {
                this.hardwareMap=hardwareMap;
                this.deviceName = deviceName;
                this.direction = Servo.Direction.FORWARD;
            }

            public Builder Direction(Servo.Direction direction) {
                this.direction = direction;
                return this;
        }

        public ServoCreator build() {
                return new ServoCreator(this);
        }
    }

    private ServoCreator(Builder builder) {
        servo = builder.hardwareMap.get(Servo.class, builder.deviceName);
        servo.setDirection(builder.direction);
    }

    public Servo getServo() {
        return servo;
    }
}
