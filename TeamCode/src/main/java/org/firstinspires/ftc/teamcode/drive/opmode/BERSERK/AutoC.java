package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

//FTC Import

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

@Autonomous(group = "BERSERK")
public class AutoC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot    = new HardwareBERSERK();
        robot.init(hardwareMap);

        double shooter_target_velo = 1800;
        double launch_angle = 0.174;
        double kicker_out = 0.7;
        double kicker_in = 0.2;
        double wobble_close = 0.45;
        double wobble_open = 1;
        double wobble_up = 0.6;
        double wobble_down = 0.16;
        long shootWait = 300;

        waitForStart();

        robot.kicker.setPosition(kicker_out);
        Pose2d startPose = new Pose2d(-63.0,50, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);
        if (isStopRequested()) return;

        //SHOOT POSITION
        Trajectory C1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .addDisplacementMarker(() -> {
                    ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                    ((DcMotorEx) robot.shooter2).setVelocity(((DcMotorEx) robot.shooter1).getVelocity());
                })
                .splineTo(new Vector2d(-3.0, 38.0), Math.toRadians(-2.0))
                .build();

        //WOBBLE C POSITION
        Trajectory C2 = drive.trajectoryBuilder(C1.end())
                .splineToConstantHeading(new Vector2d(55.0, 42.0), Math.toRadians(0.0))
                .build();

        //MOVE TOWARDS STACK
        Trajectory C3 = drive.trajectoryBuilder(C2.end(),true)
                .splineToLinearHeading(new Pose2d(0.0, 35.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .build();

        //RAM STACK (Fast Constraints)
        Trajectory C4 = drive.trajectoryBuilder(C3.end())
                .forward(12,
                        new MinVelocityConstraint(Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(80, DriveConstants.TRACK_WIDTH)
                        )
                        ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //INTAKE STACK (Slow Constraints)
        Trajectory C5 = drive.trajectoryBuilder(C4.end())
                .forward(17,
                        new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                  )
                ), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //SHOOT POSITION
        Trajectory C6 = drive.trajectoryBuilder(C5.end(),true)
                .splineToLinearHeading( new Pose2d(-3.0, 38.0, Math.toRadians(-3.0)), Math.toRadians(180))
                .build();

        //PARK
        Trajectory C7 = drive.trajectoryBuilder(C6.end())
                .forward(5)
                .build();

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        robot.flap.setPosition(launch_angle);

        //SHOOT POSITION
        drive.followTrajectory(C1);

        //SHOOT x 3
        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF SHOOTER
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //WOBBLE C POSITION
        drive.followTrajectory(C2);

        //DROP WOBBLE 1
        robot.wobble_lift.setPosition(wobble_down);
        sleep(400);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        //MOVE TOWARDS STACK
        drive.followTrajectory(C3);

        robot.wobble_lift.setPosition(wobble_up);

        //RAM STACK
        drive.followTrajectory(C4);

        robot.intake.setPower(0.8);
        robot.feeder_turn.setPower(1);
        sleep(500);

        //INTAKE STACK
        drive.followTrajectory(C5);

       ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
       ((DcMotorEx) robot.shooter2).setVelocity(((DcMotorEx) robot.shooter1).getVelocity());

        //SHOOT POSITION
        drive.followTrajectory(C6);

        //SHOOT X 3
        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF INTAKE AND SHOOTER
        robot.intake.setPower(0);
        robot.feeder_turn.setPower(0);
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //PARK
        drive.followTrajectory(C7);
    }
}
