package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_BACK_CHAMBER_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_BACK_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_BASKET_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_CHAMBER_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_VERTICAL_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_WALL_SPECIMEN_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.ClawConstants.HARD_STOP_ACTIVE;
import static org.firstinspires.ftc.teamcode.ClawConstants.HARD_STOP_INACTIVE;
import static org.firstinspires.ftc.teamcode.GamePieceType.NONE;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_RESTING_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_REVOLUTION_TIME;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MAX_HEIGHT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


@TeleOp(name = "EbTestRobotComponents", group = "Iterative Opmode")

public class EbTestRobotComponents extends OpMode {


    //Robot components
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
    private Servo hardStopServo = null;
    private Servo clawServo = null;
    private DigitalChannel intakeSwitch = null;
    private TouchSensor intakeSlideSwitch = null;
    private DigitalChannel vertSlideSwitch = null;

    //Class variables
    private GamePieceType gamePiece = NONE;
    private ClawArmPresetPosition preset = null;
    private ArrayList<Boolean> buttonValues;
    private int buttonIncrement = 0;
    private int releaseCounter = 0;
    private int grabCounter = 0;

    //Button switch toggles
    private boolean left_bumper_is_pressed = false;
    private boolean left_stick_button_pressed = false;
    private boolean dpad_left_is_pressed = false;
    private boolean dpad_right_is_pressed = false;
    private boolean right_bumper_is_pressed = false;
    private boolean g1_dpad_down_is_pressed = false;
    private boolean g1_right_stick_button_is_pressed = false;
    private boolean g1_back_is_pressed = false;
    private boolean g1_a_is_pressed = false;
    private boolean g1_b_is_pressed = false;

    private double vert_slide_power = 0;
    private double intake_slide_power = 0;

    //Robot component statuses
    private boolean intakeIsActive = false;  //DONE
    private boolean loadedIntakeIsRetracting = false;
    private boolean intakeIsLoaded = false;
    private boolean robotReadyForExchange = false;
    private boolean exchangeInProgress = false;
    private boolean specimenHangInProgress = false;
    private boolean overrideIntakeSwitch = false;
    private boolean isResetSlideMode = false;
    private boolean loadedIntakeArmAtExchange = false;
    private boolean loadedIntakeSlideAtExchange = false;
    private boolean clawReadyForExchange = false;


    private double specimenHangFinalPosition = 0;
    private double clawValue = CLAW_OPEN;
    private double clawArmValue = CLAW_ARM_EXCHANGE_POSITION;
    private double intakeArmValue = INTAKE_ARM_RESTING_POSITION;

    private String codeSection;
    //Drive component statuses
    final double DRIVE_POWER_NORMAL = 1;
    final double DRIVE_POWER_SLOW = .4;
    final int INTAKE_TICK_COUNT = 10;

    private double drivePowerPercent = DRIVE_POWER_NORMAL;


    @Override
    public void init() {
        buttonValues = new ArrayList<Boolean>();
        //Instantiate all hardware components
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
        slideVertLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake Pivot Arm Servo
        intakePivotArmServo = hardwareMap.get(Servo.class, "Intake_arm_servo");

        //Intake Frame Servo
        intakeFrameServo = hardwareMap.get(CRServo.class, "Intake_servo");
        intakeFrameServo.setDirection(DcMotorSimple.Direction.REVERSE);

        //Claw Pivot Arm Servo
        clawPivotArmServo = hardwareMap.get(Servo.class, "Claw_arm_servo");

        //Claw Servo
        clawServo = hardwareMap.get(Servo.class, "Claw_servo");

        //Intake Roller Limit Switch
        intakeSwitch = hardwareMap.get(DigitalChannel.class, "Intake_switch");

        //Intake Slide Bump Switch
        intakeSlideSwitch = hardwareMap.get(TouchSensor.class, "Intake_slide_switch");

        //Vert Slide Magnetic Switch
        vertSlideSwitch = hardwareMap.get(DigitalChannel.class, "Vert_slide_switch");

        hardStopServo = hardwareMap.get(Servo.class, "Hard_stop_servo");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        super.start();
//        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        buttonValues = new ArrayList<Boolean>();
        clawPivotArmServo.setPosition(CLAW_ARM_VERTICAL_POSITION);
        intakePivotArmServo.setPosition(IntakeArmConstants.INTAKE_ARM_EXCHANGE_POSITION);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            hardStopServo.setPosition(HARD_STOP_ACTIVE);
        }
        if (gamepad1.b) {
            hardStopServo.setPosition(HARD_STOP_INACTIVE);
        }
        if (gamepad1.y) {
            clawPivotArmServo.setPosition(CLAW_ARM_CHAMBER_HANG_POSITION);
        }
    }

}
