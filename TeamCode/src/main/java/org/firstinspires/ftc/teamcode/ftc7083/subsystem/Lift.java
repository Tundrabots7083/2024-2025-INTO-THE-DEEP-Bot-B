package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerEx;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * A lift that uses two motors and slides to raise and lower an arm and two pixel collectors.
 */
@Config
public class Lift extends SubsystemBase {
    public static int INTAKE_POSITION = 0;
    public static int SCORE_LOW_POSITION = 350;
    public static int SCORE_MEDIUM_POSITION = 700;
    public static int SCORE_HIGH_POSITION = 1100;
    public static int HANG_START_POSITION = 1525;
    public static int HANG_END_POSITION = 900;
    public static int DRONE_LAUNCH_POSITION = 0;

    public static int ACCEPTABLE_ERROR = 10;

    public static double KP = 0.009;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KG = 0.1;
    public static double INTEGRAL_LIMIT = 1;
    public static double MAX_ACCELERATION = 3000;
    public static double MAX_VELOCITY = 8000;
    public static double MIN_POWER = 0.1;

    private final Telemetry telemetry;
    private final Motor leftMotor;
    private final Motor rightMotor;

    private final PIDControllerEx leftController;
    private final PIDControllerEx rightController;

    private MotionProfile motionProfile;

    private double targetPosition = INTAKE_POSITION;

    /**
     * Instantiates a new lift that controls the motors for raising and lower the lift.
     *
     * @param hardwareMap the hardware map that contains all the robot's hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor = new Motor(hardwareMap, telemetry, "leftLift");
        initializeMotor(leftMotor);

        rightMotor = new Motor(hardwareMap, telemetry, "rightMotor");
        initializeMotor(rightMotor);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftController = new PIDControllerEx(KP, KI, KD, KG);
        leftController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        rightController = new PIDControllerEx(KP, KI, KD, KG);
        rightController.setIntegrationBounds(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        // Build the motion profile to move the lift to the target position
        motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, leftMotor.getCurrentPosition(), INTAKE_POSITION);

        telemetry.addLine("[LIFT] initialized");
    }

    /**
     * Initialize the motor used by the lift subsystem.
     *
     * @param motor the motor on the lift to be initialized.
     */
    protected void initializeMotor(Motor motor) {
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Moves the lift to the specified position.
     *
     * @param position the position to which to set the lift.
     */
    public void setPosition(final int position) {
        if (targetPosition != position) {
            targetPosition = position;
            leftController.reset();
            rightController.reset();
            motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, leftMotor.getCurrentPosition(), position);

            telemetry.addData("[LIFT] set position", position);
        }
    }

    /**
     * Adjusts the lift position in order to reach the target position.
     */
    public void execute() {
        // Read the current position of each motor
        double leftPosition = leftMotor.getCurrentPosition();
        double rightPosition = rightMotor.getCurrentPosition();

        // Get the calculated target position from our motion profile
        double intermediateTarget = motionProfile.calculatePosition();
        telemetry.addData("[LIFT] intermediate target", intermediateTarget);

        // Get the error between the two positions
        double error = Math.abs(leftPosition - targetPosition);

        double leftPower, rightPower;
        if (targetPosition == INTAKE_POSITION && error < ACCEPTABLE_ERROR) {
            leftPower = 0;
            rightPower = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            leftPower = leftController.calculate(intermediateTarget, leftPosition);
            rightPower = rightController.calculate(intermediateTarget, rightPosition);

            // Cap the motor power at 1 and -1
            leftPower = modifyMotorPower(leftPower, MIN_POWER);
            rightPower = modifyMotorPower(rightPower, MIN_POWER);
        }

        setPower(leftPower, rightPower);
    }

    /**
     * Sets the power for the two lift motors.
     *
     * @param leftPower  the left motor for the lift.
     * @param rightPower the right motor for the lift.
     */
    public void setPower(double leftPower, double rightPower) {
        // Apply the power to each motor
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("[LIFT] left power", leftPower);
        telemetry.addData("[LIFT] right power", rightPower);
        telemetry.addData("[LIFT] left position", leftMotor.getCurrentPosition());
        telemetry.addData("[LIFT] right position", rightMotor.getCurrentPosition());
    }

    /**
     * Returns an indication as to whether the lift is at the target position.
     *
     * @return <code>true</code> if the lift is at the target position;
     * <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        // Get the error between the two positions
        double error = Math.abs(leftMotor.getCurrentPosition() - targetPosition);

        final boolean finished;
        if (targetPosition == INTAKE_POSITION) {
            finished = error < ACCEPTABLE_ERROR;
        } else {
            finished = motionProfile.isAtEnd() && error < ACCEPTABLE_ERROR;
        }

        telemetry.addData("[LIFT] at target", finished);
        return finished;
    }
}
