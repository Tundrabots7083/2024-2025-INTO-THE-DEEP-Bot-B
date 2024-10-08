package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerEx;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * A lift that uses two motors and slides to raise and lower an arm and two pixel collectors.
 */
@Config
public class Lift extends SubsystemBase {
    public static double DRIVEN_GEAR_DIAMETER = 2.5;
    public static double TICKS_PER_REV = 1120.0; // AndyMark NeverRest ticks per rev
    public double GEARING = 120.0 / 24.0;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;

    public static double START_POSITION = 0.0;
    public static double ACCEPTABLE_ERROR = 0.05;

    public static double KP = 0.009;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KG = 0.1;
    public static double MIN_POWER = 0.1;

    private final Telemetry telemetry;
    private final Motor leftMotor;
    private final Motor rightMotor;

    private final PIDControllerEx leftController;
    private final PIDControllerEx rightController;

    private double targetHeight = START_POSITION;

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

        rightMotor = new Motor(hardwareMap, telemetry, "rightLift");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        initializeMotor(rightMotor);

        leftController = new PIDControllerEx(KP, KI, KD, KG);
        rightController = new PIDControllerEx(KP, KI, KD, KG);

        telemetry.addLine("[LIFT] initialized");
    }

    /**
     * Initialize the motor used by the lift subsystem.
     *
     * @param motor the motor on the lift to be initialized.
     */
    protected void initializeMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(TICKS_PER_REV);
        motorConfigurationType.setGearing(GEARING);
        motorConfigurationType.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setInchesPerRev(Math.PI * DRIVEN_GEAR_DIAMETER);
    }

    /**
     * Get the target height, in inches, for the lift.
     *
     * @return the target height, in inches
     */
    public double getTargetHeight() {
        return targetHeight;
    }

    /**
     * Set the target height, in inches, for the lift.
     *
     * @param height the target height, in inches
     */
    public void setTargetHeight(final double height) {
        if (this.targetHeight != height) {
            this.targetHeight = height;
            leftController.reset();
            rightController.reset();
            telemetry.addData("[Lift] set height", this.targetHeight);
        }
    }

    /**
     * Get the current height, in inches, for the lift.
     *
     * @return the current height, in inches
     */
    public double getCurrentHeight() {
        return Math.max(leftMotor.getInches(), rightMotor.getInches());
    }

    /**
     * Adjusts the lift position in order to reach the target position.
     */
    public void execute() {
        // Read the current position of each motor
        double leftHeight = leftMotor.getInches();
        double rightHeight = rightMotor.getInches();

        // Get the error between the two positions
        double leftError = Math.abs(leftHeight - targetHeight);
        double rightError = Math.abs(rightHeight - targetHeight);
        double error = Math.max(leftError, rightError);

        double leftPower, rightPower;
        if (targetHeight == START_POSITION && error < ACCEPTABLE_ERROR) {
            leftPower = 0;
            rightPower = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            leftPower = leftController.calculate(targetHeight, leftHeight);
            rightPower = rightController.calculate(targetHeight, rightHeight);

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
        telemetry.addData("[LIFT] left height", leftMotor.getInches());
        telemetry.addData("[LIFT] right height", rightMotor.getInches());
    }

    /**
     * Returns an indication as to whether the lift is at the target position.
     *
     * @return <code>true</code> if the lift is at the target position;
     * <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        final boolean finished = getError() < ACCEPTABLE_ERROR;

        telemetry.addData("[LIFT] at target", finished);
        return finished;
    }

    private double getError() {
        double leftError = Math.abs(leftMotor.getInches() - targetHeight);
        double rightError = Math.abs(rightMotor.getInches() - targetHeight);
        telemetry.addData("[LIFT] left error", leftError);
        telemetry.addData("[LIFT] right error", rightError);

        return Math.max(leftError, rightError);
    }
}
