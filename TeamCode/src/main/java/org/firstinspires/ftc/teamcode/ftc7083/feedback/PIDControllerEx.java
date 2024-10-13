package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

/**
 * A PID controller that uses a feed forward mechanism in its calculations.
 */
public class PIDControllerEx extends PIDController {
    private final FeedForward ff;

    /**
     * Creates a new PID controller without any FeedForward component.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     */
    public PIDControllerEx(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, 0);
    }

    /**
     * Creates a new PID controller with a constant gravity FeedForward component.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param Kg constant feed forward gravity term, added to the output of the PID calculation
     */
    public PIDControllerEx(double Kp, double Ki, double Kd, double Kg) {
        this(Kp, Ki, Kd, p -> Kg);
    }

    /**
     * Creates a new PID controller with the provided FeedForward function.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param ff the feed forward function
     */
    public PIDControllerEx(double Kp, double Ki, double Kd, FeedForward ff) {
        super(Kp, Ki, Kd);
        this.ff = ff;
    }

    @Override
    public double calculate(double reference, double state) {
        double power = super.calculate(reference, state);
        double kg = ff.calculate(state);
        FtcDashboard.getInstance().getTelemetry().addData("[PID] Kg", kg);
        return power + kg;
    }

    /**
     * Gets a string representation of this PID controller.
     *
     * @return a string representation of this PID controller
     */
    @NonNull
    @Override
    public String toString() {
        return "PIDControllerEx{" +
                super.toString() +
                "ff=" + ff +
                '}';
    }
}
