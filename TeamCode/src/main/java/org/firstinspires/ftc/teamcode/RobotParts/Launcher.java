package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    DcMotorEx launcher;
    public Launcher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void on() {
        launcher.setPower(0.4);
    }
    public void off() {
        launcher.setPower(0);
    }
}
