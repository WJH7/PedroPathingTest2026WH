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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int pathState;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotorEx intake;
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
                pathTimer.resetTimer();
                follower.followPath(startToShoot);
                pathState = 1;
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 4 || follower.atPose(shootPose, 2, 2)) {
                    pathTimer.resetTimer();
                    follower.followPath(shootToPickup);
                    pathState = 2;
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 4 || follower.atPose(openGate, 2, 2)) {
                    pathTimer.resetTimer();
                    follower.followPath(pickupToGate);
                    pathState = 3;
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 4 || follower.atPose(shootPose, 2, 2)) {
                    pathTimer.resetTimer();
                    follower.followPath(gateToShoot);
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
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
    }
    public void start() {
        pathTimer.resetTimer();
        opModeTimer.resetTimer();
        pathState = 0;
        flywheel1.setPower(0.5);
        flywheel2.setPower(0.5);
    }
    @Override
    public void loop() {
        follower.update();
        updatePathState();
        telemetry.addLine("Path Timer:" + pathTimer.getElapsedTime());
        telemetry.addLine("Path State:" + pathState);
        telemetry.update();
    }
}
