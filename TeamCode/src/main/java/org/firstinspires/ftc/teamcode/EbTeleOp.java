package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_CHANGE_TIME;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_BASKET_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHANGE_TIME;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHAMBER_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_WALL_SPECIMEN_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_ACTIVE;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXTEND;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXCHANGE;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.TIME_FOR_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.TIME_FOR_INTAKE_CLEARANCE;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_REVOLUTION_TIME;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_EXCHANGE_MARGIN_OF_ERROR;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_FAST_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MIN_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_RESTING_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_ADJUST_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD;
import static org.firstinspires.ftc.teamcode.TiltServoConstants.TILT_DOWN;
import static org.firstinspires.ftc.teamcode.TiltServoConstants.TILT_UP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_FAST_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HANG_SPEED;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HIGH_BASKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_LOW_BASKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_LOW_CHAMBER_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_EXCHANGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_FAST_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MARGIN_OF_ERROR;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_MOTOR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SPECIMEN_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_WALL_HEIGHT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "EbTeleOp", group = "Iterative Opmode")

public class EbTeleOp extends OpMode {

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
    private Servo clawServo = null;
    private Servo climbLiftServo = null;
    private DigitalChannel intakeSwitch = null;
    private DigitalChannel clawSwitch = null;
    private ColorSensor colorSensor = null;

    //Timers for servo actions
    private ElapsedTime intakeMotorTimer;
    private ElapsedTime intakeArmTimer;
    private ElapsedTime clawArmTimer;
    private ElapsedTime clawMotorTimer;

    //Class variables
    private RobotAction robotAction;
    private boolean retractionInitiated = false;
    private boolean intakeIsRunning = false;
    private boolean intakeReadyForExchange = false;
    private boolean vertReadyForExchange = false;
    private double drivePower;
    private boolean intakeIsLoaded = false;
    private boolean clawIsLoaded = false;
    private boolean sampleReadyForRelease = false;
    private boolean specimenReadyToHang = false;
    private boolean isRecalibrationMode = false;
    private boolean specimenReadyToGrab = false;
    private GamePieceType gamePiece = GamePieceType.NONE;
    private boolean isManualMode = false;
    private boolean isEndGame = false;
    private boolean isTilted = false;
    private boolean climbInitiated = false;
    private boolean clawArmIsPositioned = true;
    private boolean intakeInUse = false;
    private boolean wallGrabInitiated = false;
    private boolean hangInProgress = false;
    private boolean clawGripIsSet = true;

    private boolean resetActions = true;

    private double clawPivotDistance = 0;


    //Drive Constants
    private final double DRIVE_POWER_NORMAL = 1.0;
    private final double DRIVE_POWER_SLOW = .4;
    private final double TURN_POWER_PERCENT = 1.0;

    @Override
    public void init() {
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
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //Claw Bump Switch
        clawSwitch = hardwareMap.get(DigitalChannel.class, "Claw_switch");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        robotAction = RobotAction.IDLE;
        drivePower = DRIVE_POWER_NORMAL;

        //Set timers for Servo components
        intakeArmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeMotorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawArmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawMotorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
    /*
        Gamepad Control Assignments:
        Gamepad 1:
            Right Stick:
            Right Stick Button: STOP_AND_RESET_ENCODER
            Left Stick:     Drive (Y position) and Strafe (X position)
            Left Stick Button:
            D-Pad Up:       Cancel recalibration
            D-Pad Right:
            D-Pad Down:     Manual Slide for vertical recalibration
            D-Pad Left:     Manual Slide for intake recalibration
            A:              Drive Speed Normal
            B:              Drive Speed Slow
            X:              (end game mode) Tilt Up
            Y:              (end game mode) Tilt Down
            Right Trigger:  Turn Right
            Left Trigger:   Turn left
            Right Bumper:
            Left Bumper:
            Back (Select):  Toggle End Game

        Gamepad 2:
            Right Stick:    Manual Vertical Slide
            Right Stick Button:
            Left Stick:     Intake Operations (y position)
            Left Stick Button:
            D-Pad Up:       Wall Preset Operation
            D-Pad Right:
            D-Pad Down:
            D-Pad Left:
            A:              Lower Basket Preset Operation
            B:              Lower Chamber Preset Operation
            X:              Upper Basket Preset Operation
            Y:              Upper Chamber Preset Operation
            Right Trigger:  Manual Claw Arm Right
            Left Trigger:   Manual Claw Arm Left
            Right Bumper:
            Left Bumper:    Claw operations
            Back (Select):  Toggle Manual Mode
         */

        driveRobot();
        setRobotAction();
        executeRobotAction();
    }

    private void driveRobot() {

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        if (gamepad1.a) {
            drivePower = DRIVE_POWER_NORMAL;
        }

        if (gamepad1.b) {
            drivePower = DRIVE_POWER_SLOW;
        }

        //drive controls forward power based on Left Drive Stick's Y position
        double drive = -gamepad1.left_stick_y * drivePower;

        //turn controls left/right power based on Right and Left Triggers
        double turn;
        if (gamepad1.left_trigger > 0.0) {
            turn = -gamepad1.left_trigger * TURN_POWER_PERCENT;
        } else if (gamepad1.right_trigger > 0.0) {
            turn = gamepad1.right_trigger * TURN_POWER_PERCENT;
        } else
            turn = 0;

        //strafe controls side movement power based on Left Drive Stick's X position
        double strafe = gamepad1.left_stick_x * drivePower;

        //calculate motor power
        frontLeftPower = Range.clip((drive + turn + strafe), -1.0, 1.0);
        frontRightPower = Range.clip((drive - turn - strafe), -1.0, 1.0);
        backLeftPower = Range.clip((drive + turn - strafe), -1.0, 1.0);
        backRightPower = Range.clip((drive - turn + strafe), -1.0, 1.0);

        // Send calculated power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setRobotAction() {

        /*
        Order of Operation for setting robot actions:
        1. End Game mode
        2. Manual mode
        3. Recalibration mode
        4. Release Sample
        5. Hang specimen
        6. Align vertical slides for baskets/chambers
        7. Exchange sample
        8. Position claw and intake for exchange
        9. Use intake
        10. Align for wall grab
        11. Idle
         */
        if (isEndGame) {
            robotAction = RobotAction.RUN_END_GAME_MODE;
        } else if (isManualMode) {  //See #3 in comments
            robotAction = RobotAction.RUN_MANUAL_MODE;
        } else if (isRecalibrationMode) {
            robotAction = RobotAction.RUN_SLIDE_RECALIBRATION;
        } else if (sampleReadyForRelease) {
            robotAction = RobotAction.RUN_RELEASE_SAMPLE;
        } else if (specimenReadyToHang) {
            robotAction = RobotAction.RUN_HANG_SPECIMEN;
        } else if (clawIsLoaded) {
            robotAction = RobotAction.ALIGN_SLIDE_FOR_POINTS;
        } else if (intakeReadyForExchange && vertReadyForExchange) {
            robotAction = RobotAction.EXCHANGE_SAMPLE;
        } else if (intakeReadyForExchange) {
            robotAction = RobotAction.POSITION_CLAW_FOR_EXCHANGE;
        } else if (intakeInUse) {
            robotAction = RobotAction.USE_INTAKE;
        } else if (specimenReadyToGrab) {
            robotAction = RobotAction.GET_SPECIMEN_FROM_WALL;
        } else if (wallGrabInitiated) {
            robotAction = RobotAction.ALIGN_SLIDE_FOR_WALL;
        } else {
            robotAction = RobotAction.IDLE;
        }
    }

    private void executeRobotAction() {
        switch (robotAction) {
            case IDLE:
                if (slideVertRightMotor.getCurrentPosition() > VERT_EXCHANGE_HEIGHT ||
                        slideVertLeftMotor.getCurrentPosition() > VERT_EXCHANGE_HEIGHT) {
                    setVertSlidesByPositions(VERT_EXCHANGE_HEIGHT);
                }
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                } else if (gamepad1.dpad_up) {
                    isRecalibrationMode = true;
                } else if (gamepad2.left_stick_y != 0) {
                    useIntakeArm();
                    intakeInUse = true;
                } else if (gamepad2.right_stick_button) {
                    wallGrabInitiated = true;
                    specimenReadyToGrab = (alignVertTargets(VERT_WALL_HEIGHT, CLAW_WALL_SPECIMEN_POSITION)
                            && setClawPosition(CLAW_OPEN));
                }
                break;
            case ALIGN_SLIDE_FOR_WALL:
                if (gamepad1.back) {
                    isEndGame = true;
                    wallGrabInitiated = false;
                } else if (gamepad2.back) {
                    isManualMode = true;
                    wallGrabInitiated = false;
                } else if (gamepad2.right_stick_y != 0) {
                    useIntakeArm();
                    wallGrabInitiated = false;
                } else {
                    specimenReadyToGrab = (alignVertTargets(VERT_WALL_HEIGHT, CLAW_WALL_SPECIMEN_POSITION)
                            && setClawPosition(CLAW_OPEN));
                    wallGrabInitiated = !specimenReadyToGrab;
                }
                break;
            case GET_SPECIMEN_FROM_WALL:
                if (clawMotorTimer.time() > CLAW_CHANGE_TIME) {
                    if (gamepad1.back) {
                        specimenReadyToGrab = false;
                        isEndGame = true;
                    } else if (gamepad2.back) {
                        isManualMode = true;
                        specimenReadyToGrab = false;
                    } else if (gamepad2.left_stick_y != 0) {
                        specimenReadyToGrab = false;
                        useIntakeArm();
                    } else if (gamepad2.left_bumper) {
                        clawIsLoaded = setClawPosition(CLAW_CLOSE);
                        specimenReadyToGrab = !clawIsLoaded;
                    } else {
                        clawIsLoaded = (clawServo.getPosition() == CLAW_CLOSE);
                        specimenReadyToGrab = !clawIsLoaded;
                    }
                }
                break;
            case USE_INTAKE:
                if (gamepad1.back) {
                    intakeInUse = false;
                    isEndGame = true;
                } else {
                    useIntakeArm();
                }
                break;
            case POSITION_CLAW_FOR_EXCHANGE:
                if (gamepad1.back) {
                    isEndGame = true;
                }
                vertReadyForExchange = setClawPosition(CLAW_EXCHANGE_POSITION);
                break;
            case EXCHANGE_SAMPLE:
                clawIsLoaded = grabSampleFromExchange();
                vertReadyForExchange = !clawIsLoaded;
                intakeReadyForExchange = !clawIsLoaded;
                break;
            case ALIGN_SLIDE_FOR_POINTS:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                } else if (gamePiece == GamePieceType.SPECIMEN) {
                    if (gamepad2.a) {
                        sampleReadyForRelease = alignVertTargets(VERT_LOW_BASKET_HEIGHT, CLAW_BASKET_DROP_POSITION);
                    } else if (gamepad2.x) {
                        sampleReadyForRelease = alignVertTargets(VERT_HIGH_BASKET_HEIGHT, CLAW_BASKET_DROP_POSITION);
                    }
                } else if (gamePiece == GamePieceType.SAMPLE) {
                    if (gamepad2.b) {
                        specimenReadyToHang = alignVertTargets(VERT_LOW_CHAMBER_HEIGHT, CLAW_CHAMBER_HANG_POSITION);
                    } else if (gamepad2.y) {
                        specimenReadyToHang = alignVertTargets(VERT_HIGH_CHAMBER_HEIGHT, CLAW_CHAMBER_HANG_POSITION);
                    }
                }
                break;
            case RUN_RELEASE_SAMPLE:
                if (gamepad2.left_bumper) {
                    sampleReadyForRelease = !setClawPosition(CLAW_OPEN);
                }
                break;
            case RUN_HANG_SPECIMEN:
                if (gamepad2.left_bumper) {
                    hangInProgress = true;
                }
                specimenReadyToHang = !hangSpecimenOnChamberRung();
                break;
            case RUN_END_GAME_MODE:
                if (gamepad1.y) {
                    tiltRobot(TILT_UP);
                    isTilted = true;
                }
                if (gamepad1.x) {
                    tiltRobot(TILT_DOWN);
                    isTilted = false;
                }
                if (gamepad2.left_stick_y != 0) {
                    setVertSlidesWithLeftStick();
                    climbInitiated = true;
                }
                if (gamepad1.back && !climbInitiated) {
                    isEndGame = false;
                    if (isTilted) {
                        tiltRobot(TILT_DOWN);
                    }
                }
            case RUN_MANUAL_MODE:
                if (gamepad2.left_stick_y != 0) {
                    useIntakeArm();
                } else {
                    if (gamepad2.right_stick_y != 0) {
                        setVertSlidesWithLeftStick();
                    }
                    if (gamepad2.right_trigger != 0) {
                        clawArmIsPositioned = rotateClawArmForward();
                    } else if (gamepad2.left_trigger != 0) {
                        clawArmIsPositioned = rotateClawArmBackward();
                    }
                }
                clawArmIsPositioned = (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME);
                if (gamepad2.left_bumper) {
                    clawGripIsSet = setClawPosition((clawServo.getPosition() == CLAW_OPEN) ? CLAW_CLOSE : CLAW_OPEN);
                }
                clawGripIsSet = (clawMotorTimer.time() > CLAW_CHANGE_TIME);
                break;
            case RUN_SLIDE_RECALIBRATION:
                recalibrateSlides();
                if (gamepad1.right_stick_button) {
                    resetEncoders();
                    isRecalibrationMode = false;
                }
                if (gamepad1.dpad_up) {
                    isRecalibrationMode = false;
                }
                break;
        }

    }

    private void tiltRobot(double position) {
        if (climbLiftServo.getPosition() != position) {
            climbLiftServo.setPosition(position);
        }
    }

    private void resetEncoders() {
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean grabSampleFromExchange() {

        double motorPower = 0;

        if (!clawSwitch.getState()) {
            motorPower = VERT_SLOW_DROP;
        } else {
            return setClawPosition(CLAW_CLOSE);
        }

        slideVertLeftMotor.setPower(motorPower);
        slideVertRightMotor.setPower(motorPower);

        return false;
    }

    private void useIntakeArm() {

        intakeInUse = true;

        double motorPower = 0;
        double slidePosition = intakeSlideMotor.getCurrentPosition();
        double armPosition = intakePivotArmServo.getPosition();

        vertReadyForExchange = alignVertTargets(VERT_EXCHANGE_HEIGHT, CLAW_EXCHANGE_POSITION);

        //1.  INTAKE LOADED
        if (intakeSwitch.getState()) {  //getState == true when switch is pressed
            if (intakeIsRunning) {
                runIntakeMotor();  //if intake hasn't fully loaded the sample, allow intake to continue running
                return;
            }
            if (!retractionInitiated) {  //retractionInitiated is set to false when pivot arm is lowered
                intakeArmTimer.reset();  //reset the timer for the pivot arm to make sure intake is above bottom rail
                retractionInitiated = true;  //flag that we have begun retraction so we don't continue to loop through this code
                intakePivotArmServo.setPosition(INTAKE_ARM_EXCHANGE);  //set position of arm to EXCHANGE
            } else {
                if (intakeArmTimer.time() > TIME_FOR_INTAKE_CLEARANCE //once servo has had enough time to lift above bottom rail
                        && Math.abs(slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION) > INTAKE_EXCHANGE_MARGIN_OF_ERROR) {
                    if (slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION > INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {  // if slide is not to EXCHANGE position
                        motorPower = INTAKE_SLIDE_FAST_RETRACT_POWER;  //retract at full speed
                    } else {
                        motorPower = (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION ? -1 : 1) * INTAKE_SLIDE_SLOW_ADJUST_POWER;
                    }
                } else if (intakeArmTimer.time() > TIME_FOR_EXCHANGE_POSITION &&
                        Math.abs(slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION) < INTAKE_EXCHANGE_MARGIN_OF_ERROR) {
                    intakeReadyForExchange = true;
                    intakeInUse = false;//flag as READY
                }
            }
            //2.  SLIDE EXTENDING
        } else {
            if (gamepad2.left_stick_y > 0) { //extend arm when pushing left stick up
                if (slidePosition < INTAKE_SLIDE_MAX_EXTENSION) { //If MAX_POSITION not reached
                    motorPower = gamepad2.left_stick_y; //Set power based on y position
                }
                if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION
                        && armPosition != INTAKE_ARM_EXTEND
                        && armPosition != INTAKE_ARM_ACTIVE) { // if pivot arm is not in EXTEND or ACTIVE position and slide is beyond MIN_EXT
                    intakePivotArmServo.setPosition(INTAKE_ARM_EXTEND);  //set pivot arm to EXTEND
                }
                //3.   SLIDE RETRACTING
            } else if (gamepad2.left_stick_y < 0) {  //retract arm when pulling left stick down
                if (armPosition == INTAKE_ARM_ACTIVE) {  //if intake active
                    if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION) {  //if slide position is extended beyond MIN_EXT
                        motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER;  //set motor power to slow retract
                        runIntakeMotor();
                    }
                } else {  //if not intake active
                    if (intakeSlideMotor.getCurrentPosition() > INTAKE_SLIDE_MIN_EXTENSION) {
                        motorPower = gamepad2.left_stick_y;
                    }
                    if (intakeArmTimer.time() > TIME_FOR_INTAKE_CLEARANCE) {  //check that pivot arm is in exchange position
                        if (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION) {  //if slide position is greater than exchange
                            motorPower = gamepad2.left_stick_y;  //set power to fast retract
                        } else if (slidePosition > INTAKE_SLIDE_RESTING_POSITION) {  //if slide position is less than exchange but greater than resting
                            motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER; //set power to slow retract
                            intakeInUse = false;
                        }
                    }
                }
            }
        }

        intakeSlideMotor.setPower(motorPower);  //set intakeSlide power to motorPower

        //RAISING AND LOWERING THE INTAKE
        if (gamepad2.left_stick_button) {  //push left stick button to change intake position
            if (armPosition == INTAKE_ARM_ACTIVE) {  //if active
                intakePivotArmServo.setPosition((INTAKE_ARM_EXCHANGE)); //set to EXCHANGE
                intakeArmTimer.reset();
                //reset intakeRaiseTimer to 0
            } else if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION) {  //if not ACTIVE
                intakePivotArmServo.setPosition(INTAKE_ARM_ACTIVE); //set to ACTIVE
                retractionInitiated = false;  //flag retraction as false
            }
        }
    }

    private void runIntakeMotor() {
        double motorPower = 0;
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

    private double retractIntake(double slidePosition) {

        double motorPower = 0;
        if (!retractionInitiated) {  //retractionInitiated is set to false when pivot arm is lowered
            intakeArmTimer.reset();  //reset the timer for the pivot arm to make sure intake is above bottom rail
            retractionInitiated = true;  //flag that we have begun retraction so we don't continue to loop through this code
            intakePivotArmServo.setPosition(INTAKE_ARM_EXCHANGE);  //set position of arm to EXCHANGE
        } else {
            if (intakeArmTimer.time() > TIME_FOR_INTAKE_CLEARANCE //once servo has had enough time to lift above bottom rail
                    && Math.abs(slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION) > INTAKE_EXCHANGE_MARGIN_OF_ERROR) {
                if (slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION > INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {  // if slide is not to EXCHANGE position
                    motorPower = INTAKE_SLIDE_FAST_RETRACT_POWER;  //retract at full speed
                } else {
                    motorPower = (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION ? -1 : 1) * INTAKE_SLIDE_SLOW_ADJUST_POWER;
                }
            } else if (intakeArmTimer.time() > TIME_FOR_EXCHANGE_POSITION &&
                    Math.abs(slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION) < INTAKE_EXCHANGE_MARGIN_OF_ERROR) {
                intakeReadyForExchange = true;  //flag as READY
                robotAction = RobotAction.EXCHANGE_SAMPLE;
            }
        }
        return motorPower;
    }

    private boolean alignVertTargets(double vertPosition, double armPosition) {
        boolean vertPosSet = false;
        boolean armPosSet = false;

        if (Math.abs(vertPosition - slideVertLeftMotor.getCurrentPosition()) > VERT_MARGIN_OF_ERROR
                || Math.abs(vertPosition - slideVertRightMotor.getCurrentPosition()) > VERT_MARGIN_OF_ERROR) {
            setVertSlidesByPositions(vertPosition);
        } else {
            vertPosSet = true;
        }

        if (clawPivotArmServo.getPosition() != armPosition) {
            clawPivotDistance = Math.abs(clawPivotArmServo.getPosition() - armPosition);
            clawPivotArmServo.setPosition(armPosition);
            clawArmTimer.reset();
        } else {
            if (clawArmTimer.time() > clawPivotDistance * 180 * CLAW_ARM_CHANGE_TIME) {
                armPosSet = true;
            }
        }

        return (vertPosSet && armPosSet);
    }

    private void setVertSlidesByPositions(double position) {
        double slideLeft = slideVertLeftMotor.getCurrentPosition();
        double slideRight = slideVertRightMotor.getCurrentPosition();

        double leftOffset = slideLeft - position;
        double rightOffset = slideRight - position;

        double leftMotorPower = 0;
        double rightMotorPower = 0;

        if (Math.abs(leftOffset) > VERT_SLOW_MOTOR_THRESHOLD) {
            leftMotorPower = (leftOffset > 0) ? VERT_FAST_DROP : VERT_FAST_RAISE;
        } else if (Math.abs(leftOffset) > VERT_MARGIN_OF_ERROR) {
            leftMotorPower = (leftOffset > 0) ? VERT_SLOW_DROP : VERT_SLOW_RAISE;
        }

        if (Math.abs(rightOffset) > VERT_SLOW_MOTOR_THRESHOLD) {
            rightMotorPower = (rightOffset > 0) ? VERT_FAST_DROP : VERT_FAST_RAISE;
        } else if (Math.abs(rightOffset) > VERT_MARGIN_OF_ERROR) {
            rightMotorPower = (rightOffset > 0) ? VERT_SLOW_DROP : VERT_SLOW_RAISE;
        }

        slideVertLeftMotor.setPower(leftMotorPower);
        slideVertRightMotor.setPower(rightMotorPower);
    }

    private boolean setClawPosition(double position) {
        if (clawServo.getPosition() != position) {
            clawServo.setPosition(position);
            clawMotorTimer.reset();
            return false;
        }
        return (clawMotorTimer.time() > CLAW_CHANGE_TIME);
    }

    private boolean rotateClawArmForward() {
        double pos = clawPivotArmServo.getPosition();
        if (clawArmIsPositioned) {
            if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_EXCHANGE_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_WALL_SPECIMEN_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        } else if (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME) {
            if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_EXCHANGE_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_WALL_SPECIMEN_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    private boolean rotateClawArmBackward() {
        double pos = clawPivotArmServo.getPosition();
        if (clawArmIsPositioned) {
            if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_WALL_SPECIMEN_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_EXCHANGE_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        } else if (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME) {
            if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_WALL_SPECIMEN_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_EXCHANGE_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    private void recalibrateSlides() {
        isRecalibrationMode = true;
        double intakeMotorPower = 0;
        double vertMotorPower = 0;
        if (gamepad1.dpad_left) {
            intakeMotorPower = -INTAKE_SLIDE_SLOW_ADJUST_POWER;
        }
        intakeSlideMotor.setPower(intakeMotorPower);
        if (gamepad1.dpad_down) {
            vertMotorPower = VERT_SLOW_DROP;
        }
        slideVertLeftMotor.setPower(vertMotorPower);
        slideVertRightMotor.setPower(vertMotorPower);
    }

    private void setVertSlidesWithLeftStick() {

        double slideLeftPower = 0;
        double slideRightPower = 0;

        double leftPos = slideVertLeftMotor.getCurrentPosition();
        double rightPos = slideVertRightMotor.getCurrentPosition();

        if (leftPos > VERT_MIN_HEIGHT && leftPos < VERT_MAX_HEIGHT) {
            slideLeftPower = gamepad2.right_stick_y;
        }
        if (rightPos > VERT_MIN_HEIGHT && rightPos < VERT_MAX_HEIGHT) {
            slideRightPower = gamepad2.right_stick_y;
        }

        slideVertRightMotor.setPower(slideRightPower);
        slideVertLeftMotor.setPower(slideLeftPower);
    }

    private boolean hangSpecimenOnChamberRung() {
        //lower specimen onto chamber rung
        double pos = slideVertLeftMotor.getCurrentPosition();
        double motorPower = 0;


        slideVertLeftMotor.setPower(VERT_HANG_SPEED);
        slideVertRightMotor.setPower(VERT_HANG_SPEED);

        while (pos - slideVertLeftMotor.getCurrentPosition() < VERT_SPECIMEN_DROP) {

        }

        clawServo.setPosition(CLAW_OPEN);

        slideVertLeftMotor.setPower(0);
        slideVertRightMotor.setPower(0);

        return true;
    }

    private void runEndGameLift() {

    }

    @Override
    public void stop() {
    }

}
