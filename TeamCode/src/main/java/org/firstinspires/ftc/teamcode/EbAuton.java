package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name = "EbAuton", group = "Robot")
public class EbAuton extends LinearOpMode {

    /* Declare OpMode members. */
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
    private DigitalChannel intakeSwitch = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    private final ElapsedTime autonTimer = new ElapsedTime();

    private double headingError = 0;

    private double targetHeading = 0;
    private final double driveSpeed = 0;
    private double turnSpeed = 0;
    private final double strafeSpeed = 0;
    private final double frontLeftSpeed = 0;
    private final double frontRightSpeed = 0;
    private final double backLeftSpeed = 0;
    private final double backRightSpeed = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftTarget = 0;
    private int backRightTarget = 0;

    static final double COUNTS_PER_MOTOR_REV = 384.5;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.127;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double DRIVE_SPEED = 0.8;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.3;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.

    enum DriveType {
        FORWARD,
        STRAFE
    }

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        //Instantiate all hardware components
        //Front Left Mechanum Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_left");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Front Right Mechanum Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_right");
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Back Left Mechanum Motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_left");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Back Right Mechanum Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_right");
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Left Vertical Slide Motor
        slideVertLeftMotor = hardwareMap.get(DcMotor.class, "Slide_left");
        slideVertLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake Pivot Arm Servo
        intakePivotArmServo = hardwareMap.get(Servo.class, "Intake_arm_servo");
        intakePivotArmServo.setPosition(IntakeArmConstants.INTAKE_ARM_RESTING_POSITION);
        //Intake Frame Servo
        intakeFrameServo = hardwareMap.get(CRServo.class, "Intake_servo");
        intakeFrameServo.setDirection(DcMotorSimple.Direction.REVERSE);

        //Claw Pivot Arm Servo
        clawPivotArmServo = hardwareMap.get(Servo.class, "Claw_arm_servo");
        clawPivotArmServo.setPosition(ClawConstants.CLAW_ARM_VERTICAL_POSITION);

        //Claw Servo
        clawServo = hardwareMap.get(Servo.class, "Claw_servo");
        clawServo.setPosition(ClawConstants.CLAW_CLOSE);

        //Intake Roller Limit Switch
        intakeSwitch = hardwareMap.get(DigitalChannel.class, "Intake_switch");

        /* The next two lines define Hub orientation.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        imu.resetYaw();
        autonTimer.reset();

        runVerticalSlideToPosition(VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT);
        rotateClawArmToPosition(ClawConstants.CLAW_ARM_CHAMBER_HANG_POSITION);

        //Hang specimen on chamber
        driveForward(31);
        hangSpecimenOnHighChamber();

        //Move to color samples and push to observation zone
        driveReverse(6);
        strafeRight(24);
        driveForward(10);
        strafeRight(10);
        driveReverse(31);

    }

    private void runVerticalSlideToPosition(double position) {
        slideVertLeftMotor.setTargetPosition((int) position);
        slideVertRightMotor.setTargetPosition((int) position);
        slideVertLeftMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        slideVertRightMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
    }

    private void rotateClawArmToPosition(double position) {
        clawPivotArmServo.setPosition(position);
    }

    private void driveForward(double inches) {
        driveForward(inches, DRIVE_SPEED);
    }

    private void driveReverse(double inches) {
        driveReverse(inches, DRIVE_SPEED);
    }

    private void strafeRight(double inches) {
        strafeRight(inches, DRIVE_SPEED);
    }

    private void strafeLeft(double inches) {
        strafeLeft(inches, DRIVE_SPEED);
    }

    void hangSpecimenOnHighChamber() {
        double pos = slideVertLeftMotor.getCurrentPosition() + VerticalSlideConstants.VERT_SPECIMEN_HANG_OFFSET_DISTANCE + VerticalSlideConstants.VERT_MARGIN_OF_ERROR;
        slideVertLeftMotor.setTargetPosition((int) pos);
        slideVertRightMotor.setTargetPosition((int) pos);
        slideVertLeftMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        slideVertRightMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        while (slideVertLeftMotor.getCurrentPosition() < pos - VerticalSlideConstants.VERT_MARGIN_OF_ERROR) {
        }
        clawServo.setPosition(ClawConstants.CLAW_OPEN);
    }

    void pickUpSample() {

    }

    void putSampleInHighBucket() {

    }

    void grabSpecimenFromWall() {

    }

    void dropSample() {

    }

    public void driveForward(double inches, double speed) {
        driveStraight(speed, inches, targetHeading, DriveType.FORWARD);
    }

    public void driveReverse(double inches, double speed) {
        driveStraight(speed, -inches, targetHeading, DriveType.FORWARD);
    }

    public void strafeRight(double inches, double speed) {

    }

    private void strafeLeft(double inches, double speed) {

    }

    public void driveShortestDistance(double x, double y, double h, double speed) {

        double p1 = 0;
        double p2 = 0;

        if (x == 0 && y == 0) {
            return;
        }
        double dist = Math.sqrt(x * x + y * y);
        double angle = 0;
        if (Math.abs(y) < Math.abs(x)) {
            angle = Math.toDegrees(Math.asin(Math.abs(y) / dist));
            if ((y < 0 && x > 0) || (y > 0 && x < 0)) {
                angle = (y > 0) ? 270 + angle : 90 + angle;
            } else {
                angle = (y > 0) ? 90 - angle : 270 - angle;
            }
        } else if (Math.abs(x) < Math.abs(y)) {
            angle = Math.toDegrees(Math.asin(Math.abs(x) / dist));
            if ((x > 0 && y > 0) || (x < 0 && y < 0)) {
                angle = (x > 0) ? angle : 180 + angle;
            } else {
                angle = (x > 0) ? 180 - angle : 360 - angle;
            }
        } else {
            if (x > 0) {
                angle = (y > 0) ? 45 : 135;
            } else {
                angle = (y > 0) ? 315 : 225;
            }
        }
        angle = (h - angle) % 360;

        if (angle % 180 <= 90) {
            p1 = angle > 90 ? -1 : 1;
            p2 = ((angle % 90) - 45) / 45;
        } else {

        }


    }

    public void driveStraight(double power, double inches, double heading, DriveType type) {

        if (opModeIsActive()) {

            int multiplier = (type == DriveType.FORWARD)?1:-1;
            int moveCounts = (int) (inches * COUNTS_PER_INCH);

            frontLeftTarget = frontLeftMotor.getCurrentPosition() + moveCounts*multiplier;
            frontRightTarget = frontRightMotor.getCurrentPosition() + moveCounts*multiplier;
            backLeftTarget = backLeftMotor.getCurrentPosition() + moveCounts*multiplier;
            backRightTarget = backRightMotor.getCurrentPosition() + moveCounts*multiplier;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftMotor.setTargetPosition(frontLeftTarget);
            frontRightMotor.setTargetPosition(frontRightTarget);
            backLeftMotor.setTargetPosition(backLeftTarget);
            backRightMotor.setTargetPosition(backLeftTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            power = Math.abs(power);
            driveRobot(power, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                if (inches < 0)
                    turnSpeed *= -1.0;
                driveRobot(driveSpeed, 0, turnSpeed);
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            driveRobot(0, 0, 0);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            driveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }
        driveRobot(0, 0, 0);
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        driveRobot(0, 0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drivePower  forward motor speed
     * @param turnPower   clockwise turning motor speed.
     * @param strafePower side-strafing motor speed
     */
    private void driveRobot(double drivePower, double strafePower, double turnPower) {

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //drive controls forward power based on Left Drive Stick's Y position
        double drive = -drivePower;
        double turn = -turnPower;
        double strafe = strafePower;

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

    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", frontLeftSpeed, frontRightSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
