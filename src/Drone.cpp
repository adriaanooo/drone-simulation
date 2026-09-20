#include "Drone.hpp"
#include <cmath>

Drone::Drone(
    double mass, 
    double armLength, 
    const Matrix3& inertia, 
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
    momentArm(armLength * sin(45)),
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

void Drone::update(double escCommandFL, double escCommandFR, double escCommandRL, double escCommandRR, double dt)
{
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

    Vector3 localForce{
        0.0,
        0.0, 
        thrustFL + thrustFR + thrustRL + thrustRR
    };

    Vector3 localMoment = {
        (thrustFL + thrustRL - thrustFR - thrustRR) * momentArm,
        (thrustFL + thrustFR - thrustRL - thrustRR) * momentArm,
        torqueFL - torqueFR - torqueRL + torqueRR
    };

    angularAcceleration = inertia.inverse() * (localMoment - (angularRate.crossProduct(inertia * angularRate)));
    angularRate = angularRate + angularAcceleration * dt;
    quaternion.applyAngularRate(angularRate, dt);
    angle = quaternion.toEuler();
}