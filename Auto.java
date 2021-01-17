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


import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

@Autonomous(name = "Auto", group = "Autonomus")

public class Auto extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //DECLARE MISC VARIABLES HERE
    double powerRange = 100;
    private boolean servoArmState = true;
    private boolean contArmState = true;
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


        Components.leftDriveA = hardwareMap.get(DcMotor.class, "motor3");
        Components.leftDriveB = hardwareMap.get(DcMotor.class, "motor2");
        Components.rightDriveA = hardwareMap.get(DcMotor.class, "motor1");
        Components.rightDriveB = hardwareMap.get(DcMotor.class, "motor0");
        Components.shooterA = hardwareMap.get(DcMotor.class, "shooter0");
        Components.shooterB = hardwareMap.get(DcMotor.class, "shooter1");
        Components.contServoArm = hardwareMap.get(DcMotor.class, "arm0");
        Components.servoArm = hardwareMap.get(Servo.class, "servo1");
        Components.collector = hardwareMap.get(DcMotor.class, "collector");
        Components.groundSensor = hardwareMap.get(ColorSensor.class, "sensor0");

        telemetry.addData("Status", "Initialized ABCD");
        telemetry.update();

        Components.leftDriveA.setDirection(DcMotor.Direction.FORWARD);
        Components.leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        Components.rightDriveA.setDirection(DcMotor.Direction.REVERSE);
        Components.rightDriveB.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        ///////////////////////////////////////////////////

        //MAIN CODE
        while (opModeIsActive()) {
            
            forwardTape(0.5);
            rotateLeftAuto(0.5, 90);
            moveAuto(0.5, 5000);
            moveBackwardAuto(0.5, 750);
            rotateRightAuto(0.5, 0);
            moveBackwardAuto(0.3, 750);
            startShooting();
            rotateRightAuto(0.5, -5);
            shootOnce();
            rotateRightAuto(0.5, -10);
            shootOnce();
            rotateRightAuto(0.5, -15);
            shootOnce();
            forwardTape(0.3);
            sleep(100000);
            //forwardTape(0.3);
            /*
            telemetry.addData("Angle: ", angles.firstAngle);
            telemetry.addData("LeftDriveA: ", Components.leftDriveA.getCurrentPosition());
            telemetry.addData("LeftDriveB: ", Components.leftDriveB.getCurrentPosition());
            telemetry.addData("RightDriveA: ", Components.rightDriveA.getCurrentPosition());
            telemetry.addData("RightDriveB: ", Components.rightDriveB.getCurrentPosition());
            telemetry.update();

            */
            //mainCode();

            //todo: test heading output, change methods moveForward, moveForwardAuto, rotateLeftAuto, rotateRightAuto


    }


    }
    private void moveAuto(double power, int ticks) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float targetAngle = angles.firstAngle;
        double leftSidePower = power;
        double rightSidePower = power;

        int currTicks = 0;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while(currTicks < ticks) {
            if (angles.firstAngle < targetAngle) {
                double adjust = (targetAngle - angles.firstAngle) / 65;
                rightSidePower += adjust;
                telemetry.addData("Too far to the right. Adding to right side: ", adjust);
                //CHANGE
            } else if (angles.firstAngle > targetAngle) {
                double adjust = (angles.firstAngle - targetAngle) / 65;
                leftSidePower += adjust; // CHANGE
                telemetry.addData("Too far to the left. Adding to left side: ", adjust);
            }
            Components.leftDriveA.setPower(-leftSidePower);
            Components.leftDriveB.setPower(-leftSidePower);
            Components.rightDriveA.setPower(-rightSidePower);
            Components.rightDriveB.setPower(-rightSidePower);
            sleep(1);
            telemetry.update();
        }

    }


    private void moveForward(double power, float targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double leftSidePower = power;
        double rightSidePower = power;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(angles.firstAngle < targetAngle){
            double adjust =  (targetAngle - angles.firstAngle) / 65;
            rightSidePower += adjust;
            telemetry.addData("Too far to the right. Adding to right side: ", adjust);
            //CHANGE
        }
        else if(angles.firstAngle > targetAngle){
            double adjust = (angles.firstAngle - targetAngle) / 65;
            leftSidePower += adjust; // CHANGE
            telemetry.addData("Too far to the left. Adding to left side: ", adjust);
        }
        Components.leftDriveA.setPower(-leftSidePower);
        Components.leftDriveB.setPower(-leftSidePower);
        Components.rightDriveA.setPower(-rightSidePower);
        Components.rightDriveB.setPower(-rightSidePower);

        telemetry.update();

    }



    private void moveBackwardAuto(double power, int milliseconds) {
        Components.leftDriveA.setPower(-power);
        Components.leftDriveB.setPower(-power);
        Components.rightDriveA.setPower(-power);
        Components.rightDriveB.setPower(-power);

        sleep(milliseconds);

        Components.leftDriveA.setPower(0);
        Components.leftDriveB.setPower(0);
        Components.rightDriveA.setPower(0);
        Components.rightDriveB.setPower(0);

    }

    private void moveLeftAuto(double power, int milliseconds) {
        Components.leftDriveA.setPower(-power);
        Components.leftDriveB.setPower(power);
        Components.rightDriveA.setPower(power);
        Components.rightDriveB.setPower(-power);

        sleep(milliseconds);

        Components.leftDriveA.setPower(0);
        Components.leftDriveB.setPower(0);
        Components.rightDriveA.setPower(0);
        Components.rightDriveB.setPower(0);

    }

    private void moveRightAuto(double power, int milliseconds) {

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

    private void rotateLeftAuto(double power, float targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double leftPower = power;
        double rightPower = power;
        while(angles.firstAngle < targetAngle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle < targetAngle) {
                leftPower = -(power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.1);
                rightPower = -leftPower;
            }
            else if(angles.firstAngle > targetAngle){
                leftPower = (power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.1);
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
                leftPower = -(power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.1);
                rightPower = -leftPower;
            }
            else if(angles.firstAngle > targetAngle){
                leftPower = (power * (Math.abs(Math.abs(angles.firstAngle) - Math.abs(targetAngle)) / 65) + 0.1);
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

    private void forwardTape(double power){
        int blue;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float firstAngle = angles.firstAngle;
        boolean moving = true;
        while(moving){
            moveForward(power, firstAngle);
            blue = Components.groundSensor.blue();
            telemetry.addData("Blue", blue);
            telemetry.update();

            if(blue > 95){
                moving = false;
                break;
            }
        }
        Components.leftDriveA.setPower(0);
        Components.leftDriveB.setPower(0);
        Components.rightDriveA.setPower(0);
        Components.rightDriveB.setPower(0);
    }

    private void startShooting(){

        Components.shooterA.setPower(-0.75);
        Components.shooterB.setPower(1);

    }
    private void shootOnce(){
        Components.collector.setPower(-1);
        sleep(1000);
        Components.collector.setPower(0);
    }
    private void stopShooting(){
        Components.shooterA.setPower(0);
        Components.shooterB.setPower(0);
        Components.collector.setPower(0);
    }


}






