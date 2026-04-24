package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotParts.Flywheel;
import org.firstinspires.ftc.teamcode.RobotParts.Intake;
import org.firstinspires.ftc.teamcode.RobotParts.Launcher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="William Test TeleOp")
public class WilliamTestTele extends OpMode {
    private Follower follower;

    private Flywheel flywheel;
    private Intake intake;
    private Launcher launcher;
    private boolean intakeWasLastPressed;
    private boolean intakeToggle;
    private boolean flywheelWasLastPressed;
    private boolean flywheelToggle;
    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        flywheelToggle = false;
        intakeToggle = false;
    }
    @Override
    public void start() {
        follower.startTeleOpDrive();
    }
    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5);
        //Driver Controls
        //intake
        if ((gamepad1.left_trigger > 0.5) && !intakeWasLastPressed) {
            flywheelToggle = !flywheelToggle;
        }
        intakeWasLastPressed = gamepad1.left_trigger > 0.5;
        if (intakeToggle) {
            intake.on();
        } else {
            intake.off();
        }
        if (gamepad1.left_bumper && !flywheelWasLastPressed) {
            intakeToggle = !intakeToggle;
        }
        flywheelWasLastPressed = gamepad1.left_bumper;
        if (flywheelToggle) {
            flywheel.on();
        } else {
            flywheel.off();
        }
        if (gamepad1.right_trigger > 0.5) {
            launcher.on();
        } else {
            launcher.off();
        }
    }
}

