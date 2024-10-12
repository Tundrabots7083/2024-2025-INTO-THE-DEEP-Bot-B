package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

@TeleOp(name = "Drive Train Motor Test", group = "tests")
public class DriveTrainMotorTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");

        telemetry.update();
    }

    @Override
    public void loop() {
        double power = 0.5;
        if (gamepad1.left_bumper) {
            power *= -1;
        }
        double leftFrontPower = 0.0;
        double leftRearPower = 0.0;
        double rightFrontPower = 0.0;
        double rightRearPower = 0.0;

        if (gamepad1.dpad_up) {
            leftFrontPower = power;
        }
        if (gamepad1.dpad_down) {
            leftRearPower = power;
        }
        if (gamepad1.triangle) {
            rightFrontPower = power;
        }
        if (gamepad1.cross) {
            rightRearPower = power;
        }

        robot.mecanumDrive.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }
}
