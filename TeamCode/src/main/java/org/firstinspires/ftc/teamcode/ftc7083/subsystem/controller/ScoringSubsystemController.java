package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.ScoringSubsystem;

/**
 * Uses the gamepad controller to set the position of the arm and container servo.
 */
public class ScoringSubsystemController implements SubsystemController {
    private final ScoringSubsystem scoringSubsystem;
    private boolean manualOverride = false;

    /**
     * Initializes the arm hardware.
     *
     * @param scoringSubsystem the scoring subsystem to be controlled
     */
    public ScoringSubsystemController(ScoringSubsystem scoringSubsystem) {
        this.scoringSubsystem = scoringSubsystem;
    }

    /**
     * Sets the position of the arm, and calls the arm to update the position.
     *
     * @param gamepad1 the gamepad1 controller for the arm.
     * @param gamepad2 the gamepad2 controller for the arm.
     */
    @Override
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        Telemetry telemetry = Robot.getInstance().telemetry;
        Robot robot = Robot.getInstance();

        // Automatic update of controls
        if (gamepad1.dpad_down) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.INTAKE);
            manualOverride = false;
        } else if (gamepad1.dpad_left) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.SCORE_LOW);
            manualOverride = false;
        } else if (gamepad1.dpad_right) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.SCORE_MEDIUM);
            manualOverride = false;
        } else if (gamepad1.dpad_up) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.SCORE_HIGH);
            manualOverride = false;
        } else if (gamepad2.y) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.LAUNCH_DRONE);
            manualOverride = false;
        } else if (gamepad1.x) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.HANG_START);
            manualOverride = false;
        } else if (gamepad1.y) {
            scoringSubsystem.setPosition(ScoringSubsystem.Position.HANG_END);
            manualOverride = false;
        }

        if (manualOverride || (gamepad1.right_trigger != 0.0 || gamepad1.left_trigger != 0.0)) {
            manualOverride = true;
            if (gamepad1.left_trigger > 0.0) {
                robot.lift.setPower(gamepad1.left_trigger, gamepad1.left_trigger);
                telemetry.addData("[LIFT] Power", gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                robot.lift.setPower(-gamepad1.right_trigger, -gamepad1.right_trigger);
                telemetry.addData("[LIFT] Power", -gamepad1.right_trigger);
            } else {
                robot.lift.setPower(0.0, 0.0);
                telemetry.addData("[ARM] Power", 0.0);
            }
            return;
        }

        scoringSubsystem.execute();
    }
}

