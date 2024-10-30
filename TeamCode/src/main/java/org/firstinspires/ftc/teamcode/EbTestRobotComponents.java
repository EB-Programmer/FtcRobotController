package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_REVOLUTION_TIME;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MAX_HEIGHT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class EbTestRobotComponents extends OpMode {

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor slideVertRightMotor = null;
    private DcMotor slideVertLeftMotor = null;
    private DcMotor intakeSlideMotor = null;
    private Servo intakePivotArmServo = null;
    private CRServo intakeFrameServo = null;
    private Servo clawPivotArmServo = null;
    private Servo clawServo = null;
    private Servo climbLiftServo = null;
    private DigitalChannel intakeSwitch = null;
    private ColorSensor colorSensor = null;
    private ElapsedTime intakeMotorTimer;
    private boolean intakeIsRunning;
    private boolean usingIntake;



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //Front Left Mechanum Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_left");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Front Right Mechanum Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_right");

        //Back Left Mechanum Motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_left");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Back Right Mechanum Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_right");

        //Left Vertical Slide Motor
        slideVertLeftMotor = hardwareMap.get(DcMotor.class, "Slide_left");
        slideVertLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideVertLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        //Intake Pivot Arm Servo
        intakePivotArmServo = hardwareMap.get(Servo.class, "Intake_arm_servo");
        intakePivotArmServo.setPosition(.6);

        //Intake Frame Servo
        intakeFrameServo = hardwareMap.get(CRServo.class, "Intake_servo");

        //Claw Pivot Arm Servo
        clawPivotArmServo = hardwareMap.get(Servo.class, "Claw_arm_servo");

        //Claw Servo
        clawServo = hardwareMap.get(Servo.class, "Claw_servo");

        //Climb Lift Servo
        climbLiftServo = hardwareMap.get(Servo.class, "Climb_lift_servo");

        //Intake Color Sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "Color_sensor");

        //Intake Roller Limit Switch
        intakeSwitch = hardwareMap.get(DigitalChannel.class, "Intake_switch");

        intakeMotorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        usingIntake = false;
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        final double POWER_PERCENT = .25;

        double leftVertSlidePower = (gamepad1.dpad_left?1:0)*POWER_PERCENT;
        double rightVertSlidePower = (gamepad1.dpad_right?1:0)*POWER_PERCENT;
        double clawSlidePower = (gamepad1.dpad_up?1:0)*POWER_PERCENT;
        double frontLeftMotorPower = (gamepad1.y?1:0)*POWER_PERCENT;
        double frontRightMotorPower = (gamepad1.b?1:0)*POWER_PERCENT;
        double backLeftMotorPower = (gamepad1.x?1:0)*POWER_PERCENT;
        double backRightMotorPower = (gamepad1.a?1:0)*POWER_PERCENT;

        if(usingIntake) {
            if (gamepad1.right_bumper) {
                usingIntake = false;
            } else {
                runIntakeMotor();
            }
        } else {
            if (gamepad1.right_bumper) {
                usingIntake = true;
                runIntakeMotor();
            }
        }

        if (slideVertLeftMotor.getCurrentPosition()<VERT_MAX_HEIGHT) {
            slideVertLeftMotor.setPower(leftVertSlidePower);
        }

        if (slideVertRightMotor.getCurrentPosition()<VERT_MAX_HEIGHT) {
            slideVertRightMotor.setPower(rightVertSlidePower);
        }

        if (intakeSlideMotor.getCurrentPosition()< INTAKE_SLIDE_MAX_EXTENSION) {
            intakeSlideMotor.setPower(clawSlidePower);
        }

        frontLeftMotor.setPower(frontLeftMotorPower);
        frontRightMotor.setPower(frontRightMotorPower);
        backLeftMotor.setPower(backLeftMotorPower);
        backRightMotor.setPower(backRightMotorPower);

    }

    private void runIntakeMotor() {
        double motorPower = 0;
        usingIntake = true;
        if (intakeIsRunning) {
            if (intakeMotorTimer.time() < INTAKE_MOTOR_REVOLUTION_TIME) {
                motorPower = INTAKE_MOTOR_POWER;
            } else {
                intakeIsRunning = false;
                intakeMotorTimer.reset();
            }
        } else {
            if (intakeMotorTimer.time() > INTAKE_MOTOR_REVOLUTION_TIME) {
                motorPower = INTAKE_MOTOR_POWER;
                intakeMotorTimer.reset();
            }
        }
        intakeFrameServo.setPower(motorPower);
    }

}
