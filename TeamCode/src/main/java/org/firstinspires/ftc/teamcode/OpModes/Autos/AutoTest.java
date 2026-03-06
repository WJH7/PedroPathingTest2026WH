package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {

    }
    PathState pathState;
    private final Pose pose1 = new Pose(20.10405177242047, 122.63915064449796, Math.toRadians(135));
    private final Pose pose2 = new Pose(56.148939736477416, 87.10958304886576, Math.toRadians(135));
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        start();

    }
}
