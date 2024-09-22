package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.MotionProfile;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * The arm mechanism that moves the pixel collector from the intake position to the scoring
 * position and vice-versa.
 */
@Config
public class Arm extends SubsystemBase {
    public static int INTAKE_POSITION = 0;
    public static int SCORE_POSITION = 2700;

    public static double KP = 0.003;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double MAX_ACCELERATION = 3000;
    public static double MAX_VELOCITY = 5000;
    public static double MIN_POWER = 0.16;
    public static double ACCEPTABLE_ERROR = 10;

    private final Motor motor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    private double targetPosition = INTAKE_POSITION;
    private MotionProfile motionProfile;

    /**
     * Instantiates a new arm for the robot.
     *
     * @param hardwareMap the mapping of the hardware on the robot.
     * @param telemetry   the telemetry used to output information to the user.
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = new Motor(hardwareMap, telemetry, "armMotor");
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(KP, KI, KD);
        motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, 0, INTAKE_POSITION);

        telemetry.addLine("[ARM] initialized");
    }

    /**
     * Sets the target position for the arm.
     *
     * @param position the target position for the arm.
     */
    public void setPosition(final int position) {
        if (targetPosition != position) {
            targetPosition = position;
            pidController.reset();
            motionProfile = new MotionProfile(MAX_ACCELERATION, MAX_VELOCITY, motor.getCurrentPosition(), position);

            telemetry.addData("[ARM] set position", position);
        }
    }

    /**
     * Adjusts the arm position to reach the target position.
     */
    public void execute() {
        // Read the current position of each motor
        double position = motor.getCurrentPosition();

        // Get the calculated target position from our motion profile
        double intermediateTarget = motionProfile.calculatePosition();
        telemetry.addData("[ARM] intermediate target", intermediateTarget);

        // Get the error between the two positions
        double error = Math.abs(position - targetPosition);

        final double power;
        if (targetPosition == INTAKE_POSITION && error < ACCEPTABLE_ERROR) {
            power = 0;
        } else {
            // Calculate the power for each motor using the PID controllers
            power = pidController.calculate(intermediateTarget, position);
        }

        setPower(power);
    }

    /**
     * Sets the power for the two lift motors.
     *
     * @param power the power for the arm motor.
     */
    public void setPower(double power) {
        // Apply the power to the arm motor
        motor.setPower(power);

        telemetry.addData("[ARM] target", targetPosition);
        telemetry.addData("[ARM] power", power);
        telemetry.addData("[ARM] position", motor.getCurrentPosition());
    }

    /**
     * Returns an indication as to whether the arm is at the target position.
     *
     * @return <code>true</code> if the arm is at the target position;
     * <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        // Get the error between the two positions
        double error = Math.abs(motor.getCurrentPosition() - targetPosition);

        final boolean finished;
        if (targetPosition == INTAKE_POSITION) {
            finished = error < ACCEPTABLE_ERROR;
        } else {
            finished = motionProfile.isAtEnd() && error > ACCEPTABLE_ERROR;
        }

        telemetry.addData("[ARM] at target", finished);
        return finished;
    }
}
