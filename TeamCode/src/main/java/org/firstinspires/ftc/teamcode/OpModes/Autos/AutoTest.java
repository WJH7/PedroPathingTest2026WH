package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int pathState;
    private final Pose startPose = new Pose(20, 122, Math.toRadians(135));
    private final Pose shootPose = new Pose(56.148939736477416, 87.10958304886576, Math.toRadians(135));
    private final Pose ballPickup1 = new Pose(15, 84, Math.toRadians(180));
    private final Pose openGate = new Pose(15, 70, Math.toRadians(180));
    private final Pose gatePathControlPoint = new Pose(35, 76);
    private PathChain startToShoot;
    private PathChain shootToPickup;
    private PathChain pickupToGate;
    private PathChain gateToShoot;
    public void pathBuilder() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        shootToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, ballPickup1))
                .setLinearHeadingInterpolation(ballPickup1.getHeading(), ballPickup1.getHeading())
                .build();
        pickupToGate = follower.pathBuilder()
                .addPath(new BezierCurve(ballPickup1, gatePathControlPoint, openGate))
                .setLinearHeadingInterpolation(ballPickup1.getHeading(), openGate.getHeading())
                .build();
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(openGate, shootPose))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPose.getHeading())
                .build();
    }
    public void updatePathState() {
        switch (pathState) {
            case 0:
                follower.followPath(startToShoot);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(shootToPickup);
                    pathTimer.resetTimer();
                    pathState = 2;
                } else {
                    telemetry.addLine("Follower Is Busy");
                }
                break;
            case 2:
                if (!follower.isBusy() && follower.atPose(openGate, 2, 2) || pathTimer.getElapsedTime() > 4) {
                    follower.followPath(pickupToGate);
                    pathTimer.resetTimer();
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy() && follower.atPose(shootPose, 2, 2) || pathTimer.getElapsedTime() > 4) {
                    follower.followPath(gateToShoot);
                    pathTimer.resetTimer();
                }
                break;
        }
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        pathBuilder();
        follower.setPose(startPose);
    }
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        pathState = 0;

    }
    @Override
    public void loop() {
        follower.update();
        updatePathState();
        telemetry.addLine("Path Timer:" + pathTimer);
        telemetry.update();
    }
}
