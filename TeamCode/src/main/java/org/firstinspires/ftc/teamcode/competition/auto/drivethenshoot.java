package org.firstinspires.ftc.teamcode.competition.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "drive then shoot", group = "competitive")

public class drivethenshoot extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        DcMotorEx frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontL");
        DcMotorSimple backLeftDrive = hardwareMap.get(DcMotorSimple.class, "backL");
        DcMotorEx frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontR");
        DcMotorEx backRightDrive = hardwareMap.get(DcMotorEx.class, "backR");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Servo intake = hardwareMap.get(Servo.class, "intake");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        frontLeftDrive.setPower(-0.5);
        frontRightDrive.setPower(-0.5);
        backLeftDrive.setPower(-0.5);
        backRightDrive.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("moving", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        shooter.setPower(0.55);
        while (opModeIsActive() && (runtime.seconds() < 4)) {
            telemetry.addData("revving", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        intake.setPosition(1.0);
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("shooting", "%4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        shooter.setPower(0.0);
        intake.setPosition(0.5);
    }
}