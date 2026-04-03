package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Flywheel;

@TeleOp(name="Flywheel Tuner")
public class FlywheelTuner extends OpMode {
    private Flywheel flywheel;
    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double currentTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double D = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.0001};
    int stepIndex = 1;
    @Override
    public void init() {
        flywheel = new  Flywheel(hardwareMap, P, D, F);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        //switch target velocity
        if (gamepad1.y) {
            if (currentTargetVelocity == highVelocity) {
                currentTargetVelocity = lowVelocity;
            } else {
                currentTargetVelocity = highVelocity;
            }
        }
        //change stepSize
        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        //change F by stepSize
        if (gamepad1.dpad_left) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_right) {
            F -= stepSizes[stepIndex];
        }
        //change P by stepSize
        if (gamepad1.dpad_up) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down) {
            P -= stepSizes[stepIndex];
        }
        //change D by stepSize
        if (gamepad1.x) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.a) {
            D -= stepSizes[stepIndex];
        }
        flywheel.updatePIDF(P, D, F);
        //setVelocity
        flywheel.setVelocity(currentTargetVelocity);
        double curVelocity = (flywheel.getVelocity1() + flywheel.getVelocity2())/2;
        double flywheelError = currentTargetVelocity - curVelocity;
        //telemetry
        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", flywheelError);
        telemetry.addLine("----------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Tuning D", "%.4f (X/A)", D);
        telemetry.addData("StepSize", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
