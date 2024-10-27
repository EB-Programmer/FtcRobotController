package org.firstinspires.ftc.teamcode;

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
    private ColorSensor colorSensor = null;

    //Class variables
    private RobotAction robotAction;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime intakeRaiseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean retractionInitiated = false;
    private boolean intakeIsRunning = false;
    private boolean readyForExchange = false;

    //Drive Constants
    private final double DRIVE_POWER_PERCENT = 0.75;
    private final double TURN_POWER_PERCENT = 1.0;
    private final double STRAFE_POWER_PERCENT = 1.0;

    //Vertical slide constants
    private final double MAX_SLIDE_HEIGHT = 3175;
    private final double MIN_SLIDE_HEIGHT = 15;
    private final double MID_SLIDE_HEIGHT = 2500;
    private final double SLIDE_POWER_PERCENT = .9;

    //Horizontal intake slide constants

    //Intake pivot arm constants
    private final double TIME_FOR_INTAKE_CLEARANCE = 1.0;
    private final double TIME_FOR_EXCHANGE_POSITION = 1.5;

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

        //Gyro
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robotAction = RobotAction.IDLE;
        runtime.reset();
        intakeRaiseTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    /*
        Gamepad Control Assignments:
        Gamepad 1:
            Right Stick:
            Left Stick:  Drive (Y position) and Strafe (X position)
            D-Pad Up:
            D-Pad Right:
            D-Pad Down:
            D-Pad Left
            A:              Drive Speed Normal
            B:              Drive Speed Slow
            X:
            Y:
            Right Trigger:  Turn Right
            Left Trigger:   Turn left

        Gamepad 2:
            Right Stick:
            Left Stick:     Intake Operations (y position)
            D-Pad Up:       Manual Vertical Slide Up
            D-Pad Right:
            D-Pad Down:     Manual Vertical Slide Down
            D-Pad Left:
            A:              Lower Basket Preset Operation
            B:              Lower Chamber Preset Operation
            X:              Upper Basket Preset Operation
            Y:              Upper Chamber Preset Operation
            Right Trigger:
            Left Trigger:
            Right Bumper:   Manual claw close
            Left Bumper:    Manual claw open

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

        //drive controls forward power based on Left Drive Stick's Y position
        double drive = -gamepad1.left_stick_y * DRIVE_POWER_PERCENT;

        //turn controls left/right power based on Right and Left Triggers
        double turn;
        if (gamepad1.left_trigger > 0.0) {
            turn = -gamepad1.left_trigger * TURN_POWER_PERCENT;
        } else if (gamepad1.right_trigger > 0.0) {
            turn = gamepad1.right_trigger * TURN_POWER_PERCENT;
        } else
            turn = 0;

        //strafe controls side movement power based on Left Drive Stick's X position
        double strafe = gamepad1.left_stick_x * STRAFE_POWER_PERCENT;

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
        We may want to prevent selecting different actions if an action
        is in progress.  For example, we may want to complete the intake run
        moving on to a new action
         */
        if (robotAction == RobotAction.USE_INTAKE) {
            return;
        }
        /*
        if a sample is loaded and ready for exchange, then we don't want to be able to run intake
         */
        if (gamepad2.right_stick_y != 0) {
            robotAction = RobotAction.USE_INTAKE;
        } else if (gamepad2.dpad_up || gamepad2.dpad_down) {
            robotAction = RobotAction.RUN_VERT_SLIDE_MANUALLY;
        }
    }

    private void executeRobotAction() {
        switch (robotAction) {
            case IDLE:
                break;
            case USE_INTAKE:
                runIntake();
                break;
            case RUN_VERT_SLIDE_MANUALLY:
                setVerticalSlides();
                break;
        }

    }

    private void runIntake() {

        final double INTAKE_SLIDE_MAX_EXTENSION = 1600;
        final double INTAKE_SLIDE_RESTING_POSITION = 15;
        final double INTAKE_SLIDE_EXCHANGE_POSITION = 100;
        final double INTAKE_SLIDE_MIN_EXTENSION = 400;
        final double INTAKE_SLIDE_EXTEND_POWER = 1.0;
        final double INTAKE_SLIDE_SLOW_ADJUST_POWER = .25;
        final double INTAKE_SLIDE_SLOW_RETRACT_POWER = -.25;
        final double INTAKE_SLIDE_FAST_RETRACT_POWER = -1;
        final double INTAKE_EXCHANGE_MARGIN_OF_ERROR = 32;

        //we need to set these numbers once we have a servo
        final double INTAKE_ARM_FINAL = 0;
        final double INTAKE_ARM_EXCHANGE = .2;
        final double INTAKE_ARM_EXTEND = .5;
        final double INTAKE_ARM_ACTIVE = 1;

        double motorPower = 0;
        double slidePosition = intakeSlideMotor.getCurrentPosition();
        double armPosition = intakePivotArmServo.getPosition();

        //1.  INTAKE LOADED
        if (intakeSwitch.getState()) {  //getState == true when switch is pressed
            if (!retractionInitiated) {  //retractionInitiated is set to false when pivot arm is lowered
                intakeRaiseTimer.reset();  //reset the timer for the pivot arm to make sure intake is above bottom rail
                retractionInitiated = true;  //flag that we have begun retraction so we don't continue to loop through this code
                intakePivotArmServo.setPosition(INTAKE_SLIDE_EXCHANGE_POSITION);  //set position of arm to EXCHANGE
            } else if (intakeRaiseTimer.time() > TIME_FOR_INTAKE_CLEARANCE //once servo has had enough time to lift above bottom rail
                    && Math.abs(slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION) > INTAKE_EXCHANGE_MARGIN_OF_ERROR) {
                if (slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION > 300) {  // if slide is not to EXCHANGE position
                    motorPower = INTAKE_SLIDE_FAST_RETRACT_POWER;  //retract at full speed
                } else {
                    motorPower = (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION ? -1 : 1) * INTAKE_SLIDE_SLOW_ADJUST_POWER;
                }
            } else if (intakeRaiseTimer.time() > TIME_FOR_EXCHANGE_POSITION &&
                    Math.abs(slidePosition - INTAKE_SLIDE_EXCHANGE_POSITION) < INTAKE_EXCHANGE_MARGIN_OF_ERROR) {
                readyForExchange = true;  //flag as READY
                robotAction = RobotAction.EXCHANGE_FROM_INTAKE;
            }
            //2.  SLIDE EXTENDING
        } else {
            if (gamepad2.left_stick_y > 0) { //extend arm when pushing left stick up
                if (slidePosition < INTAKE_SLIDE_MAX_EXTENSION) { //If MAX_POSITION not reached
                    motorPower = gamepad2.left_stick_y; //Set power based on y position
                }
                if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION
                        && (armPosition != INTAKE_ARM_EXTEND
                        || armPosition != INTAKE_ARM_ACTIVE)) { // if pivot arm is not in EXTEND or ACTIVE position and slide is beyond MIN_EXT
                    intakePivotArmServo.setPosition(INTAKE_ARM_EXTEND);  //set pivot arm to EXTEND
                }
                //3.   SLIDE RETRACTING
            } else if (gamepad2.left_stick_y < 0) {  //retract arm when pulling left stick down
                if (armPosition == INTAKE_ARM_ACTIVE) {  //if intake active
                    if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION) {  //if slide position is extended beyond MIN_EXT
                        motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER;  //set motor power to slow retract
                    /*
                    NEED CODE FOR RUNNING MOTOR ON INTAKE
                     */
                    }
                } else {  //if not intake active
                    if (intakeRaiseTimer.time() > TIME_FOR_INTAKE_CLEARANCE) {  //check that pivot arm is in exchange position
                        if (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION) {  //if slide position is greater than exchange
                            motorPower = INTAKE_SLIDE_FAST_RETRACT_POWER;  //set power to fast retract
                        } else if (slidePosition > INTAKE_SLIDE_RESTING_POSITION) {  //if slide position is less than exchange but greater than resting
                            motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER; //set power to slow retract
                            robotAction = RobotAction.IDLE;
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
                intakeRaiseTimer.reset();  //reset intakeRaiseTimer to 0
            } else if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION) {  //if not ACTIVE
                intakePivotArmServo.setPosition(INTAKE_ARM_ACTIVE); //set to ACTIVE
                retractionInitiated = false;  //flag retraction as false
            }
        }
    }

    private void setVerticalSlides() {

        double slideLeftPower;
        double slideRightPower;
        slideLeftPower = 0;
        slideRightPower = 0;

        if (gamepad2.dpad_up && slideVertLeftMotor.getCurrentPosition() < 100) {
            slideLeftPower = 0.62;
            slideRightPower = 0.62;
        }


        if (gamepad2.y && slideVertLeftMotor.getCurrentPosition() < MAX_SLIDE_HEIGHT) {
            slideLeftPower = .4;
            slideRightPower = .4;
        }

        if (gamepad2.x && slideVertLeftMotor.getCurrentPosition() < MID_SLIDE_HEIGHT) {
            slideLeftPower = .4;
            slideRightPower = .4;
        }

        if (gamepad2.a && slideVertLeftMotor.getCurrentPosition() > MIN_SLIDE_HEIGHT) {
            slideLeftPower = -.4;
            slideRightPower = -.4;
        }


        slideVertRightMotor.setPower(slideRightPower);
        slideVertLeftMotor.setPower(slideLeftPower);
    }

    private void executeIntakeAction() {
        //extend horizontal slide

        //flip intake to active position

        //activate intake motors, monitor roller switch

        //flip intake to loaded position

        //retract horizontal slide
    }


    private void executeClawAction() {
        /*
        Four scenarios to handle:
            1.  Take Sample from Intake  (use grabSampleFromIntake)
            2.  Drop Sample into Basket
            3.  Grab specimen from observation zone
            4.  Hang specimen from chamber rung

        */

        //check state variable and run appropriate code

    }

    private void hangSpecimenOnChamberRung() {
        //check positioning or set positioning (may use an enum for control state)

        //lower specimen onto chamber rung

        //release claw

        //set claw to idle position
    }

    private void grabSpecimenFromWall() {
        //check for action from controller

        // get arm into position

        // close claw

        //lift to hang position
    }

    private void dropSampleIntoBasket() {
        //check if sample is loaded into claw or run on button push

        //raise vertical slide to correct position (lower or higher basket)

        //rotate arm over to position over basket

        //release sample from claw

        //reset to idle position

        //lower slide
    }

    private void grabSampleFromIntake() {
        //check if intake is in load position and has a sample

        // lower claw arm into proper position

        // close claw around sample

        //release sample from intake
    }

    private void runEndGameLift() {

    }

    //Test robot components independently
    private void runTestMode() {


    }


    @Override
    public void stop() {
    }

}
