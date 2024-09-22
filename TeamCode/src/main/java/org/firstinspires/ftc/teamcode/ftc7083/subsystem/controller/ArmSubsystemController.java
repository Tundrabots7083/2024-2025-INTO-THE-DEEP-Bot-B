package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

public class ArmSubsystemController  {
    Wrist wrist;

    ArmSubsystemController(Wrist wrist){ wrist = wrist; }

    public void liftArm(double angle){
        //shoulder.rotate(angle);
        wrist.setPitch(180 - angle);
    }
}
