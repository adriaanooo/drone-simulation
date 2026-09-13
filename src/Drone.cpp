#include "Drone.hpp"
#include <cmath>

Drone::Drone(
    double mass, 
    double armLength, 
    const Vector3& inertia, 
    Motor& motorFL, 
    Motor& motorFR, 
    Motor& motorRL, 
    Motor& motorRR,
    ESC& escFL,
    ESC& escFR,
    ESC& escRL,
    ESC& escRR
)
    : mass(mass),
    armLength(armLength),
    inertia(inertia),
    motorFL(motorFL),
    motorFR(motorFR),
    motorRL(motorRL),
    motorRR(motorRR),
    escFL(escFL),
    escFR(escFR),
    escRL(escRL),
    escRR(escRR)
{
}

void Drone::update(double escCommandFL, double escCommandFR, double escCommandRL, double escCommandRR, double dt) {
    escFL.update(escCommandFL, dt);
    escFR.update(escCommandFR, dt);
    escRL.update(escCommandRL, dt);
    escRR.update(escCommandRR, dt);

    motorFL.update(escFL.getVoltageOutput(), dt);
    motorFR.update(escFR.getVoltageOutput(), dt);
    motorRL.update(escRL.getVoltageOutput(), dt);
    motorRR.update(escRR.getVoltageOutput(), dt);

    double thrustFL = motorFL.getThrust();
    double thrustFR = motorFR.getThrust();
    double thrustRL = motorRL.getThrust();
    double thrustRR = motorRR.getThrust();

    double torqueFL = motorFL.getTorque();
    double torqueFR = motorFR.getTorque();
    double torqueRL = motorRL.getTorque();
    double torqueRR = motorRR.getTorque();

    double netThrust = thrustFL + thrustFR + thrustRL + thrustRR;

    // X
    acceleration.x = (netThrust / mass) * 
    (
        cos(angle.z) * sin(angle.y) * cos(angle.x)
        + sin(angle.z) * sin(angle.x)
    );
    velocity.x += acceleration.x * dt;
    position.x += velocity.x * dt;

    // Y
    acceleration.y =
        (netThrust / mass) *
        (
            sin(angle.z) * sin(angle.y) * cos(angle.x)
            - cos(angle.z) * sin(angle.x)
        );
    velocity.y += acceleration.y * dt;
    position.y += velocity.y * dt;

    // Z
    acceleration.z =
        (netThrust / mass) *
        (
            cos(angle.y) * cos(angle.x)
        ) - 9.81;
    velocity.z += acceleration.z * dt;
    position.z += velocity.z * dt;

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