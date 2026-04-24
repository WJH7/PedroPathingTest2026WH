package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotorEx intake;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void on() {
        intake.setPower(1);
    }
    public void off() {
        intake.setPower(0);
    }
}

