#include "Drone.hpp"
#include <cmath>

Drone::Drone(double armLength, double inertiaXX, double inertiaYY, double inertiaZZ, Motor& motorFL, Motor& motorFR, Motor& motorRL, Motor& motorRR, IMU& imu)
    : armLength(armLength),
    inertiaXX(inertiaXX),
    inertiaYY(inertiaYY),
    inertiaZZ(inertiaZZ),
    motorFL(motorFL),
    motorFR(motorFR),
    motorRL(motorRL),
    motorRR(motorRR),
    imu(imu)
{
}

void Drone::update(double voltageFL, double voltageFR, double voltageRL, double voltageRR, double dt) {
    motorFL.update(voltageFL, dt);
    motorFR.update(voltageFR, dt);
    motorRL.update(voltageRL, dt);
    motorRR.update(voltageRR, dt);

    double thrustFL = motorFL.getThrust();
    double thrustFR = motorFR.getThrust();
    double thrustRL = motorRL.getThrust();
    double thrustRR = motorRR.getThrust();

    double torqueFL = motorFL.getTorque();
    double torqueFR = motorFR.getTorque();
    double torqueRL = motorRL.getTorque();
    double torqueRR = motorRR.getTorque();

    // Roll
    double netRollTorque = (thrustFL + thrustRL - thrustFR - thrustRR) * armLength * sin(M_PI / 4);

    rollAcceleration = netRollTorque / inertiaXX;
    rollRate += rollAcceleration * dt;
    rollAngle += rollRate * dt;

    // Pitch
    double netPitchTorque = (thrustFL + thrustFR - thrustRL - thrustRR) * armLength * sin(M_PI / 4);

    pitchAcceleration = netPitchTorque / inertiaYY;
    pitchRate += pitchAcceleration * dt;
    pitchAngle += pitchRate * dt;

    // Yaw
    double netYawTorque = (torqueFL + torqueRR - torqueFR - torqueRL);

    yawAcceleration = netYawTorque / inertiaZZ;
    yawRate += yawAcceleration * dt;
    yawAngle += yawRate * dt;

    // Sensor simulation
    Vector3 angularRate(rollRate, pitchRate, yawRate);
    imu.update(angularRate, dt);
}