package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Flywheel {
    public DcMotorEx flywheel1, flywheel2;
    public double flywheelP = 64;
    public double flywheelD = 26;
    public double flywheelF = 14;
    public Flywheel(HardwareMap hardwareMap){
        //MAIN INIT STUFF
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
          flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
          flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

          //PIDF STUFF
          PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, 0, flywheelD, flywheelF);
          flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
          flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

          //Set flywheel P and F

    }


    public void on() {
        flywheel1.setPower(0.6);
        flywheel2.setPower(0.6);
        return;
    }
    public void off() {
        flywheel1.setPower(0);
        flywheel2.setPower(0);
        return;
    }
    public void updatePIDF(double newP, double newD, double newF) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(newP, 0, newD, newF);
        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        return;
    }
    public void setVelocity(double vel) {
        flywheel1.setVelocity(vel);
        flywheel2.setVelocity(vel);
    }
    public double getVelocity1() {
        return flywheel1.getVelocity();
    }
    public double getVelocity2() {
        return flywheel2.getVelocity();
    }
}
