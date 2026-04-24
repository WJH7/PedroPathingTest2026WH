package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotParts.Flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="William Test TeleOp")
public class WilliamTestTele extends OpMode {
    private Follower follower;

    private Flywheel flywheel;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
    }
}
