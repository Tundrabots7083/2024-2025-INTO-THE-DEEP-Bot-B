package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Lift;

@Config
public class LiftTest extends OpMode {
    public static double LIFT_HEIGHT = 0.0;
    private Lift lift;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        lift.setTargetHeight(LIFT_HEIGHT);
        lift.execute();
        telemetry.addData("Target Length", lift.getTargetHeight());
        telemetry.addData("Current Length", lift.getCurrentHeight());
        telemetry.update();
    }
}
