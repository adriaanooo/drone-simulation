#include "Drone.hpp"
#include <cmath>

Drone::Drone(double armLength, const Vector3& inertia, Motor& motorFL, Motor& motorFR, Motor& motorRL, Motor& motorRR)
    : armLength(armLength),
    inertia(inertia),
    motorFL(motorFL),
    motorFR(motorFR),
    motorRL(motorRL),
    motorRR(motorRR)
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

    angularAcceleration.x = netRollTorque / inertia.x;
    angularRate.x += angularAcceleration.x * dt;
    angle.x += angularRate.x * dt;

    // Pitch
    double netPitchTorque = (thrustFL + thrustFR - thrustRL - thrustRR) * armLength * sin(M_PI / 4);

    angularAcceleration.y = netPitchTorque / inertia.y;
    angularRate.y += angularAcceleration.y * dt;
    angle.y += angularRate.y * dt;

    // Yaw
    double netYawTorque = (torqueFL + torqueRR - torqueFR - torqueRL);

    angularAcceleration.z = netYawTorque / inertia.z;
    angularRate.z += angularAcceleration.z * dt;
    angle.z += angularRate.z * dt;
}