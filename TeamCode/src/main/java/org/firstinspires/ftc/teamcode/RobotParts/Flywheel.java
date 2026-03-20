package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    public DcMotorEx flywheel1, flywheel2;
    public Flywheel(HardwareMap hardwareMap){
          flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
          flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
          flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void on() {
        flywheel1.setPower(0.6);
        flywheel2.setPower(0.6);
        return;
    }
    public void off() {
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }

}
