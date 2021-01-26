/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Robot Move", group = "Movement Modable")

public class Robot extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //DECLARE MISC VARIABLES HERE
    double powerRange = 100;
    double multiplier = -1;

    private boolean servoArmState = true;
    //////////////////////////////

    //DECLARE OBJECTS HERE
    Move movement = new Move();
    //////////////////////////////


    @Override
    public void runOpMode() {
        //INITALIZATION
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        boolean slowMode = false;


        Components.leftDriveA = hardwareMap.get(DcMotor.class, "motor0");
        Components.leftDriveB = hardwareMap.get(DcMotor.class, "motor1");
        Components.rightDriveA = hardwareMap.get(DcMotor.class, "motor2");
        Components.rightDriveB = hardwareMap.get(DcMotor.class, "motor3");
        Components.shooterA = hardwareMap.get(DcMotor.class, "shooter0");
        Components.shooterB = hardwareMap.get(DcMotor.class, "shooter1");
        Components.collector = hardwareMap.get(DcMotor.class, "collector");
        Components.contServoArm = hardwareMap.get(DcMotor.class, "arm0");
        Components.servoArm = hardwareMap.get(Servo.class, "servo1");
        telemetry.addData("Status", "Initialized ABCD");
        telemetry.update();

        Components.leftDriveA.setDirection(DcMotor.Direction.FORWARD);
        Components.leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        Components.rightDriveA.setDirection(DcMotor.Direction.REVERSE);
        Components.rightDriveB.setDirection(DcMotor.Direction.REVERSE);
        Components.contServoArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        ///////////////////////////////////////////////////

        //MAIN CODE
        while (opModeIsActive()) {
            double leftStickX = -gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = -gamepad1.right_stick_x;
            double rightStickY = -gamepad1.right_stick_y;
            double YPowerRS = Range.clip(rightStickY, -powerRange, powerRange);
            double XPowerLS = Range.clip(leftStickX, -powerRange, powerRange);
            double XPowerRS = Range.clip(rightStickX, -powerRange, powerRange);
            double YPowerLS = Range.clip(leftStickY, -powerRange, powerRange);

            movement.move(-leftStickX * multiplier, leftStickY * multiplier, rightStickX , rightStickY * multiplier, -XPowerLS * multiplier, YPowerLS * multiplier, XPowerRS, YPowerRS * multiplier, slowMode);

            armMove();
            shooterTurn();
            collectorTurn();


            if(gamepad1.left_bumper == true) {
                slowMode = !slowMode;
                sleep(200);
            }


            //todo rebuild onto robot


        ///////////////////////////////////////////////////////////////////////////////////////////
        runtime.reset();

    }


    }


    private void collectorTurn(){
        double leftTrigger = gamepad1.left_trigger;
        double forwardPower = Range.clip(leftTrigger, -100, 100);
        Components.collector.setPower(forwardPower);

        double rightTrigger = gamepad1.right_trigger;
        double backwardPower = Range.clip(rightTrigger, -100, 100);
        Components.collector.setPower(-backwardPower);

    }
    private void shooterTurn(){
        if(gamepad1.y == true){
            Components.shooterA.setPower(0);
            Components.shooterB.setPower(0);
            sleep(300);
        }

        if(gamepad1.right_bumper == true){

            Components.collector.setPower(1);
            Components.shooterA.setPower(0.4);
            Components.shooterB.setPower(-0.4);
            sleep(150);
            Components.collector.setPower(0);
            Components.shooterA.setPower(0);
            Components.shooterB.setPower(-0);
            moveLeftAuto(0.4, 600);
            rotateLeftAuto(0.75, 18);


            Components.shooterA.setPower(-0.75);
            Components.shooterB.setPower(1);
        }

        if(gamepad1.dpad_left == true){

            Components.collector.setPower(1);
            Components.shooterA.setPower(0.4);
            Components.shooterB.setPower(-0.4);
            sleep(250);
            Components.collector.setPower(0);
            Components.shooterA.setPower(0);
            Components.shooterB.setPower(-0);


            Components.shooterA.setPower(-1);
            Components.shooterB.setPower(1);
        }

    }

    private void armMove(){
        if(gamepad1.a == true) {
            if(servoArmState)
                Components.servoArm.setPosition(100);
            else
                Components.servoArm.setPosition(0);
            servoArmState = !servoArmState;
            sleep(300);
        }

        if(gamepad1.dpad_up){
            Components.contServoArm.setPower(-0.5);
        }
        else if(gamepad1.dpad_down) {
            Components.contServoArm.setPower(0.5);
        }
        else {
            Components.contServoArm.setPower(0);
        }
    }

    private void rotateLeftAuto(double power, float targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double leftPower = power;
        double rightPower = power;
        while(angles.firstAngle < targetAngle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle < targetAngle) {
                leftPower = -(power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.2);
                rightPower = -leftPower;
            }
            else if(angles.firstAngle > targetAngle){
                leftPower = (power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.2);
                rightPower = -leftPower;
            }
            Components.leftDriveA.setPower(-leftPower);
            Components.leftDriveB.setPower(-leftPower);
            Components.rightDriveA.setPower(-rightPower);
            Components.rightDriveB.setPower(-rightPower);
            telemetry.addData("Angle: ", angles.firstAngle);
            telemetry.update();
        }
    }

    private void rotateRightAuto(double power, float targetAngle) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double leftPower = power;
        double rightPower = power;
        while(angles.firstAngle > targetAngle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle < targetAngle) {
                leftPower = -(power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.2);
                rightPower = -leftPower;
            }
            else if(angles.firstAngle > targetAngle){
                leftPower = (power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.2);
                rightPower = -leftPower;
            }
            Components.leftDriveA.setPower(-leftPower);
            Components.leftDriveB.setPower(-leftPower);
            Components.rightDriveA.setPower(-rightPower);
            Components.rightDriveB.setPower(-rightPower);
            telemetry.addData("Angle: ", angles.firstAngle);
            telemetry.update();
        }

    }

    private void moveLeftAuto(double power, int milliseconds) {

        Components.leftDriveA.setPower(power);
        Components.leftDriveB.setPower(-power);
        Components.rightDriveA.setPower(-power);
        Components.rightDriveB.setPower(power);

        sleep(milliseconds);

        Components.leftDriveA.setPower(0);
        Components.leftDriveB.setPower(0);
        Components.rightDriveA.setPower(0);
        Components.rightDriveB.setPower(0);

    }

    private void checkOrientation(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;
        if(heading > -70 && heading < 70){
            multiplier = -1;
            telemetry.addData("Mult -1", heading);
        }
        else{
            telemetry.addData("Mult 1", heading);
            multiplier = 1;
        }

        telemetry.update();
    }

}



