package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_CHANGE_TIME;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_BASKET_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHANGE_TIME;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHAMBER_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_WALL_SPECIMEN_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_ACTIVE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_CLEARANCE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXCHANGE_TIME;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_CLEARANCE_TIME;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_REVOLUTION_TIME;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.IntakeDirection.*;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_FAST_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_RECALIBRATE_SPEED;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_RESTING_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_ADJUST_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_TRAWL_SPEED;
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
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLIDE_RECALIBRATE_SPEED;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_MOTOR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SPECIMEN_DROP_HEIGHT;
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
    private GamePieceType gamePiece = GamePieceType.NONE;

    private double drivePower;
    private double clawPivotDistance = 0;

    //action indicators
    private boolean isRetractedIntake = false;
    private boolean isActiveIntake = false;
    private boolean isReadyForExchangeIntake = false;
    private boolean isReadyForExchangeVert = false;
    private boolean isLoadedClaw = false;
    private boolean isReadyForReleaseSample = false;
    private boolean isReadyToHangSpecimen = false;
    private boolean isRecalibrationMode = false;
    private boolean isReadyToGrabSpecimen = false;
    private boolean isManualMode = false;
    private boolean isEndGame = false;
    private boolean isTilted = false;
    private boolean isInitiatedClimb = false;
    private boolean isPositionedClawArm = true;
    private boolean isInUseIntake = false;
    private boolean isInitiatedSpecimenGrab = false;
    private boolean isRunningIntake = false;
    private boolean isInProgressHang = false;
    private boolean isSetClawGrip = true;

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
        } else if (isReadyForReleaseSample) {
            robotAction = RobotAction.RUN_RELEASE_SAMPLE;
        } else if (isReadyToHangSpecimen) {
            robotAction = RobotAction.RUN_HANG_SPECIMEN;
        } else if (isLoadedClaw) {
            robotAction = RobotAction.ALIGN_SLIDE_FOR_POINTS;
        } else if (isReadyForExchangeIntake && isReadyForExchangeVert) {
            robotAction = RobotAction.EXCHANGE_SAMPLE;
        } else if (isReadyForExchangeIntake) {
            robotAction = RobotAction.POSITION_CLAW_FOR_EXCHANGE;
        } else if (isInUseIntake) {
            robotAction = RobotAction.USE_INTAKE;
        } else if (isReadyToGrabSpecimen) {
            robotAction = RobotAction.GET_SPECIMEN_FROM_WALL;
        } else if (isInitiatedSpecimenGrab) {
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
                    isReadyForExchangeVert = hasSetClawArmPosition(VERT_EXCHANGE_HEIGHT)
                            && hasSetVertPosition(VERT_EXCHANGE_HEIGHT);
                }
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                } else if (gamepad1.dpad_up) {
                    isRecalibrationMode = true;
                } else if (gamepad2.left_stick_y != 0) {
                    useIntakeArm(gamepad2.left_stick_y);
                    isInUseIntake = true;
                } else if (gamepad2.right_stick_button) {
                    isInitiatedSpecimenGrab = true;
                    isReadyToGrabSpecimen =
                            hasSetVertPosition(VERT_WALL_HEIGHT)
                                    && hasSetClawArmPosition(CLAW_WALL_SPECIMEN_POSITION)
                                    && hasOpenedClaw();
                }
                break;
            case ALIGN_SLIDE_FOR_WALL:
                if (gamepad1.back) {
                    isEndGame = true;
                    isInitiatedSpecimenGrab = false;
                } else if (gamepad2.back) {
                    isManualMode = true;
                    isInitiatedSpecimenGrab = false;
                } else if (gamepad2.left_stick_y != 0) {
                    useIntakeArm(gamepad2.left_stick_y);
                    isInitiatedSpecimenGrab = false;
                } else {
                    isReadyToGrabSpecimen =
                            hasSetVertPosition(VERT_WALL_HEIGHT)
                                    && hasSetClawArmPosition(CLAW_WALL_SPECIMEN_POSITION)
                                    && hasOpenedClaw();
                    isInitiatedSpecimenGrab = !isReadyToGrabSpecimen;
                }
                break;
            case GET_SPECIMEN_FROM_WALL:
                if (clawMotorTimer.time() > CLAW_CHANGE_TIME) {
                    if (gamepad1.back) {
                        isReadyToGrabSpecimen = false;
                        isEndGame = true;
                    } else if (gamepad2.back) {
                        isManualMode = true;
                        isReadyToGrabSpecimen = false;
                    } else if (gamepad2.left_stick_y != 0) {
                        isReadyToGrabSpecimen = false;
                        useIntakeArm(gamepad2.left_stick_y);
                    } else if (gamepad2.left_bumper) {
                        isLoadedClaw = hasClosedClaw();
                        isReadyToGrabSpecimen = !isLoadedClaw;
                    } else {
                        isLoadedClaw = (clawServo.getPosition() == CLAW_CLOSE);
                        isReadyToGrabSpecimen = !isLoadedClaw;
                    }
                }
                break;
            case USE_INTAKE:
                if (gamepad1.back) {
                    isInUseIntake = false;
                    isEndGame = true;
                } else {
                    useIntakeArm(gamepad2.left_stick_y);
                    if (gamepad2.right_stick_button) {
                        isActiveIntake = hasActivatedIntake(!isActiveIntake);
                    }
                }
                break;
            case POSITION_CLAW_FOR_EXCHANGE:
                if (gamepad1.back) {
                    isEndGame = true;
                }
                isReadyForExchangeVert =
                        hasSetVertPosition(VERT_EXCHANGE_HEIGHT)
                                && hasSetClawArmPosition(CLAW_EXCHANGE_POSITION);
                break;
            case EXCHANGE_SAMPLE:
                isLoadedClaw = grabSampleFromExchange();
                isReadyForExchangeVert = !isLoadedClaw;
                isReadyForExchangeIntake = !isLoadedClaw;
                break;
            case ALIGN_SLIDE_FOR_POINTS:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                } else if (gamePiece == GamePieceType.SPECIMEN) {
                    if (gamepad2.a) {
                        isReadyForReleaseSample =
                                hasSetVertPosition(VERT_LOW_BASKET_HEIGHT)
                                        && hasSetClawArmPosition(CLAW_BASKET_DROP_POSITION);
                    } else if (gamepad2.x) {
                        isReadyForReleaseSample =
                                hasSetVertPosition(VERT_HIGH_BASKET_HEIGHT)
                                        && hasSetClawArmPosition(CLAW_BASKET_DROP_POSITION);
                    }
                } else if (gamePiece == GamePieceType.SAMPLE) {
                    if (gamepad2.b) {
                        isReadyToHangSpecimen =
                                hasSetVertPosition(VERT_LOW_CHAMBER_HEIGHT)
                                        && hasSetClawArmPosition(CLAW_CHAMBER_HANG_POSITION);
                    } else if (gamepad2.y) {
                        isReadyToHangSpecimen =
                                hasSetVertPosition(VERT_HIGH_CHAMBER_HEIGHT)
                                        && hasSetClawArmPosition(CLAW_CHAMBER_HANG_POSITION);
                    }
                }
                break;
            case RUN_RELEASE_SAMPLE:
                if (gamepad2.left_bumper) {
                    isReadyForReleaseSample = !hasOpenedClaw();
                }
                break;
            case RUN_HANG_SPECIMEN:
                if (gamepad2.left_bumper) {
                    isInProgressHang = true;
                }
                isReadyToHangSpecimen = !hasHungSpecimen();
                break;
            case RUN_END_GAME_MODE:
                if (gamepad1.y) {
                    isTilted = hasTiltedRobot(TILT_UP);
                }
                if (gamepad1.x) {
                    isTilted = hasTiltedRobot(TILT_DOWN);
                }
                if (gamepad2.left_stick_y != 0) {
                    setVertSlidesManually(gamepad2.left_stick_y);
                    isInitiatedClimb = true;
                }
                if (gamepad1.back && !isInitiatedClimb) {
                    isEndGame = false;
                    if (isTilted) {
                        isTilted = hasTiltedRobot(TILT_DOWN);
                    }
                }
            case RUN_MANUAL_MODE:
                if (gamepad2.left_stick_y != 0) {
                    useIntakeArm(gamepad2.left_stick_y);
                } else {
                    if (gamepad2.right_stick_y != 0) {
                        setVertSlidesManually(gamepad2.right_stick_y);
                    }
                    if (gamepad2.right_trigger != 0) {
                        isPositionedClawArm = hasRotatedClawForward();
                    } else if (gamepad2.left_trigger != 0) {
                        isPositionedClawArm = hasRotatedClawBackward();
                    }
                }
                isPositionedClawArm = (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME);
                if (gamepad2.left_bumper) {
                    if (clawServo.getPosition() == CLAW_OPEN) {
                        isSetClawGrip = hasClosedClaw();
                    } else {
                        isSetClawGrip = hasOpenedClaw();
                    }
                }
                isSetClawGrip = (clawMotorTimer.time() > CLAW_CHANGE_TIME);
                break;
            case RUN_SLIDE_RECALIBRATION:
                if (gamepad1.dpad_down || gamepad1.dpad_right) {
                    recalibrateSlides(gamepad1.dpad_right, gamepad2.dpad_down);
                }
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

    private boolean hasTiltedRobot(double position) {
        boolean b = (position == TILT_UP);
        if (climbLiftServo.getPosition() != position) {
            climbLiftServo.setPosition(position);
        }
        return b;
    }

    private void resetEncoders() {
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
        NEED TO FIGURE OUT THIS CODE ONCE THE BUILD IS COMPLETE
     */
    private boolean grabSampleFromExchange() {
        double motorPower = 0;

        if (!clawSwitch.getState()) {
            motorPower = VERT_SLOW_DROP;
        } else {
            return hasClosedClaw();
        }

        slideVertLeftMotor.setPower(motorPower);
        slideVertRightMotor.setPower(motorPower);

        return false;
    }

    private void useIntakeArm(double power) {

        /*
        1.  Intake is loaded;
        2.  Intake is active;
        3.  Intake is inactive
         */
        isInUseIntake = true;

        double motorPower = 0;
        double slidePosition = intakeSlideMotor.getCurrentPosition();
        double armPosition = intakePivotArmServo.getPosition();

        /*
        if intake is in use, then the vertical arm and the claw should be set to the exchange position
         */
        isReadyForExchangeVert =
                hasSetVertPosition(VERT_EXCHANGE_HEIGHT)
                        && hasSetClawArmPosition(CLAW_EXCHANGE_POSITION);

        //1.  INTAKE LOADED
        /*
        When a sample is pulled into the intake, the roller switch will move to the CLOSED position.
        Intake motor needs to run until the intake rotation completes.  Once it is complete, the
        intake arm should automatically lift and the intake slide should retract.
         */
        if (intakeSwitch.getState()) {  //this is the roller switch, true is CLOSED
            if (isRunningIntake) {
                runIntakeMotor(INTAKE);  //if intake hasn't fully loaded the sample, allow intake to continue running
                return;
            } else if (!hasWrongColorInIntake()) {
                runIntakeMotor(EJECT);
            } else if (!isRetractedIntake) {
                fastRetractIntake();
            } else {
                isReadyForExchangeIntake = hasSetIntakeArmPosition(INTAKE_ARM_EXCHANGE_POSITION, INTAKE_ARM_EXCHANGE_TIME);
                isInUseIntake = !isReadyForExchangeIntake;
            }
        } else if (isActiveIntake) {
            /*
            2.  Intake Active
            Slide moves at set speed when intake is active so that the drivers can trawl for
            samples.
             */
            runIntakeMotor(INTAKE);
            if (power > 0) {
                if (slidePosition < INTAKE_SLIDE_MAX_EXTENSION) {
                    motorPower = power;
                }
            } else {
                if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) {
                    motorPower = INTAKE_SLIDE_TRAWL_SPEED;
                }
            }
        } else {
            /*
            3.  Intake Inactive
             */
            if (power > 0) {
                if (slidePosition < INTAKE_SLIDE_MAX_EXTENSION) { //If MAX_POSITION not reached
                    motorPower = power; //Set power based on y position
                }
                if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE
                        && armPosition != INTAKE_ARM_CLEARANCE_POSITION
                        && armPosition != INTAKE_ARM_ACTIVE_POSITION) { // if pivot arm is not in EXTEND or ACTIVE position and slide is beyond MIN_EXT
                    intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);  //set pivot arm to EXTEND
                }
                //3.   SLIDE RETRACTING
            } else if (power < 0) {  //retract arm when pulling left stick down
                if (slidePosition > INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {
                    motorPower = power;
                } else if (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION) {
                    motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER;
                } else {
                    isInUseIntake = hasSetIntakeArmPosition(INTAKE_ARM_EXCHANGE_POSITION, INTAKE_ARM_EXCHANGE_TIME);
                }
            }
        }
        intakeSlideMotor.setPower(motorPower);  //set intakeSlide power to motorPower
    }


    private void runIntakeMotor(IntakeMotorConstants.IntakeDirection intakeDirection) {
        double motorPower = 0;
        double multiplier = (intakeDirection == INTAKE ? 1 : -1);
        if (isRunningIntake) {
            if (intakeMotorTimer.time() < INTAKE_MOTOR_REVOLUTION_TIME) {
                motorPower = INTAKE_MOTOR_POWER;
            } else {
                isRunningIntake = false;
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

    private void fastRetractIntake() {

        double motorPower = 0;
        double pos = intakeSlideMotor.getCurrentPosition();

        boolean isAtClearanceHeight = hasSetIntakeArmPosition(INTAKE_ARM_CLEARANCE_POSITION, INTAKE_ARM_CLEARANCE_TIME);

        if (pos > INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) {
            motorPower = INTAKE_SLIDE_FAST_RETRACT_POWER;
        } else if (isAtClearanceHeight && pos > INTAKE_SLIDE_EXCHANGE_POSITION + INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {
            motorPower = INTAKE_SLIDE_FAST_RETRACT_POWER;
        } else if (isAtClearanceHeight && pos > INTAKE_SLIDE_EXCHANGE_POSITION + INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR) {
            motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER;
        } else if (pos < INTAKE_SLIDE_EXCHANGE_POSITION - INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR) {
            motorPower = INTAKE_SLIDE_SLOW_ADJUST_POWER;
        } else if (Math.abs(pos - INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) < INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR) {
            isRetractedIntake = true;
            return;
        }
        intakeSlideMotor.setPower(motorPower);
    }

    /*
    NEED TO FIGURE OUT THIS CODE ONCE BUILD IS COMPLETE
     */
    private boolean hasWrongColorInIntake() {
        return false;
    }

    private boolean hasSetVertPosition(double position) {
        if (Math.abs(position - slideVertLeftMotor.getCurrentPosition()) > VERT_MARGIN_OF_ERROR
                || Math.abs(position - slideVertRightMotor.getCurrentPosition()) > VERT_MARGIN_OF_ERROR) {
            setVertSlidesByPositions(position);
            return false;
        } else {
            return true;
        }
    }

    private boolean hasSetClawArmPosition(double position) {
        if (clawPivotArmServo.getPosition() != position) {
            clawPivotArmServo.setPosition(position);
            clawArmTimer.reset();
            return false;
        }
        return (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME);
    }

    private boolean hasOpenedClaw() {
        if (clawServo.getPosition() != CLAW_OPEN) {
            clawServo.setPosition(CLAW_OPEN);
            clawMotorTimer.reset();
            return false;
        } else {
            return clawMotorTimer.time() > CLAW_CHANGE_TIME;
        }
    }

    private boolean hasClosedClaw() {
        if (clawServo.getPosition() != CLAW_CLOSE) {
            clawServo.setPosition(CLAW_CLOSE);
            clawMotorTimer.reset();
            return false;
        } else {
            return clawMotorTimer.time() > CLAW_CHANGE_TIME;
        }
    }

    private boolean hasSetIntakeArmPosition(double position, double time) {
        if (intakePivotArmServo.getPosition() != position) {
            intakePivotArmServo.setPosition(position);
            intakeArmTimer.reset();
            return false;
        }
        return intakeArmTimer.time() > time;
    }

    private boolean hasActivatedIntake(boolean activate) {
        if (activate) {
            if (intakeSlideMotor.getCurrentPosition() < INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) {
                return false;
            }
            return hasSetIntakeArmPosition(INTAKE_ARM_ACTIVE_POSITION, INTAKE_ARM_CLEARANCE_TIME);
        }
        hasSetIntakeArmPosition(INTAKE_ARM_CLEARANCE_POSITION, INTAKE_ARM_CLEARANCE_TIME);
        return false;
    }

    private boolean hasHungSpecimen() {
        //lower specimen onto chamber rung
        double pos = slideVertLeftMotor.getCurrentPosition();
        double motorPower = 0;

        if (pos > VERT_SPECIMEN_DROP_HEIGHT) {
            motorPower = VERT_HANG_SPEED;
        }

        slideVertLeftMotor.setPower(motorPower);
        slideVertRightMotor.setPower(motorPower);

        if (pos > VERT_SPECIMEN_DROP_HEIGHT) {
            return false;
        } else {
            return hasOpenedClaw();
        }

    }

    private boolean hasRotatedClawForward() {
        double pos = clawPivotArmServo.getPosition();
        if (isPositionedClawArm) {
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

    private boolean hasRotatedClawBackward() {
        double pos = clawPivotArmServo.getPosition();
        if (isPositionedClawArm) {
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

    private void recalibrateSlides(boolean recalIntake, boolean recalVert) {
        isRecalibrationMode = true;
        double intakeMotorPower = 0;
        double vertMotorPower = 0;
        if (recalIntake) {
            intakeMotorPower = INTAKE_SLIDE_RECALIBRATE_SPEED;
        }
        intakeSlideMotor.setPower(intakeMotorPower);
        if (recalVert) {
            vertMotorPower = VERT_SLIDE_RECALIBRATE_SPEED;
        }
        slideVertLeftMotor.setPower(vertMotorPower);
        slideVertRightMotor.setPower(vertMotorPower);
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

    private void setVertSlidesManually(double power) {

        double slideLeftPower = 0;
        double slideRightPower = 0;

        double leftPos = slideVertLeftMotor.getCurrentPosition();
        double rightPos = slideVertRightMotor.getCurrentPosition();

        if (leftPos > VERT_MIN_HEIGHT && leftPos < VERT_MAX_HEIGHT) {
            slideLeftPower = power;
        }
        if (rightPos > VERT_MIN_HEIGHT && rightPos < VERT_MAX_HEIGHT) {
            slideRightPower = power;
        }

        slideVertRightMotor.setPower(slideRightPower);
        slideVertLeftMotor.setPower(slideLeftPower);
    }


    @Override
    public void stop() {
    }
}
