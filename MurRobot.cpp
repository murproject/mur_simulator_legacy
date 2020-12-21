//
// Copyright (c) 2008-2017 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Material.h>

#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/Constraint.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <iostream>
#include <Urho3D/IO/FileSystem.h>

#include "MurRobot.h"

MurRobot::MurRobot(Context* context) :
    LogicComponent(context),
    steering_(0.0f)
{
    // Only the physics update event is needed: unsubscribe from the rest for optimization
    SetUpdateEventMask(USE_FIXEDUPDATE);
}

void MurRobot::RegisterObject(Context* context)
{
    context->RegisterFactory<MurRobot>();

    URHO3D_ATTRIBUTE("Controls Yaw", float, controls_.yaw_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Controls Pitch", float, controls_.pitch_, 0.0f, AM_DEFAULT);
    URHO3D_ATTRIBUTE("Steering", float, steering_, 0.0f, AM_DEFAULT);
    // Register wheel node IDs as attributes so that the wheel nodes can be reaquired on deserialization. They need to be tagged
    // as node ID's so that the deserialization code knows to rewrite the IDs in case they are different on load than on save
    URHO3D_ATTRIBUTE("Left Motor Node", unsigned, leftMotorID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Right Motor Node", unsigned, rightMotorID_, 0, AM_DEFAULT | AM_NODEID);
    URHO3D_ATTRIBUTE("Up Motor Node", unsigned, upMotorID_, 0, AM_DEFAULT | AM_NODEID);

}

void MurRobot::ApplyAttributes()
{
    // This function is called on each Serializable after the whole scene has been loaded. Reacquire wheel nodes from ID's
    // as well as all required physics components
    Scene* scene = GetScene();

    left_ = scene->GetNode(leftMotorID_);
    right_ = scene->GetNode(rightMotorID_);
    up_ = scene->GetNode(upMotorID_);
    hullBody_ = node_->GetComponent<RigidBody>();

    GetWheelComponents();
}

void MurRobot::FixedUpdate(float timeStep)
{
    Quaternion hullRot = hullBody_->GetRotation();
    leftBody_->ApplyForce(hullRot * Vector3::FORWARD * motorA_ * 10);
    rightBody_->ApplyForce(hullRot * Vector3::FORWARD * motorB_ * 10);
    upBody_->ApplyForce(hullRot * Vector3::UP *  motorC_ * 10.0f);

    hullBody_->ApplyForce(hullRot*Vector3::DOWN * 0.3, (hullBody_->GetCenterOfMass() - Vector3(0,-4,0)) + hullRot.Inverse().PitchAngle() * Vector3::FORWARD);
    hullBody_->ApplyForce(hullRot*Vector3::DOWN * 0.3, (hullBody_->GetCenterOfMass() - Vector3(0,-4,0)) + hullRot.Inverse().RollAngle() * Vector3::LEFT);

}

void MurRobot::Init()
{
    // This function is called only from the main program when initially creating the vehicle, not on scene load
    node_->SetScale(Vector3(0.2f, 0.2f, 0.2f));
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    hullBody_ = node_->CreateComponent<RigidBody>();
    hullBody_->SetMass(3.0f);
    hullBody_->SetLinearDamping(0.7f); // Some air resistance
    hullBody_->SetAngularDamping(0.7f);
    hullBody_->SetCollisionLayer(1);

    CollisionShape* hullShape = node_->CreateComponent<CollisionShape>();
    hullShape->SetBox(Vector3(10,10,10));
    hullShape->SetPosition(hullBody_->GetPosition());

    StaticModel* hullObject = node_->CreateComponent<StaticModel>();
    hullObject->SetModel(cache->GetResource<Model>("Models/MUR model.mdl"));
    //hullObject->SetMaterial(cache->GetResource<Material>("Materials/08 - Default.xml"));
    File loadFile(context_, GetSubsystem<FileSystem>()->GetProgramDir() + "Data/Materials/MUR model.txt", FILE_READ);

    hullObject->ApplyMaterialList();
    hullObject->SetCastShadows(true);

    InitWheel("Left", hullBody_->GetCenterOfMass() - Vector3(3,-4, -2.5f), left_, leftMotorID_);
    InitWheel("Right", hullBody_->GetCenterOfMass() - Vector3(-3,-4, -2.5f), right_, rightMotorID_);
    InitWheel("Up", hullBody_->GetCenterOfMass() - Vector3(0,-4.5f,0), up_, upMotorID_);


    GetWheelComponents();
}

void MurRobot::InitWheel(const String& name, const Vector3& offset, WeakPtr<Node>& wheelNode, unsigned& wheelNodeID)
{
    ResourceCache* cache = GetSubsystem<ResourceCache>();

    // Note: do not parent the wheel to the hull scene node. Instead create it on the root level and let the physics
    // constraint keep it together
    wheelNode = GetScene()->CreateChild(name);
    wheelNode->SetPosition(node_->LocalToWorld(offset));
    wheelNode->SetScale(Vector3(0.0f, 0.0f, 0.0f));

    // Remember the ID for serialization
    wheelNodeID = wheelNode->GetID();

    StaticModel* wheelObject = wheelNode->CreateComponent<StaticModel>();
    RigidBody* wheelBody = wheelNode->CreateComponent<RigidBody>();
    CollisionShape* wheelShape = wheelNode->CreateComponent<CollisionShape>();
    Constraint* wheelConstraint = wheelNode->CreateComponent<Constraint>();

//    wheelObject->SetModel(cache->GetResource<Model>("Models/Cylinder.mdl"));
//    wheelObject->SetMaterial(cache->GetResource<Material>("Materials/Stone.xml"));
    wheelObject->SetCastShadows(false);
    wheelShape->SetSphere(1.0f);
    wheelBody->SetFriction(1.0f);
    wheelBody->SetMass(0.01f);

    wheelConstraint->SetConstraintType(CONSTRAINT_POINT);
    wheelConstraint->SetOtherBody(GetComponent<RigidBody>()); // Connect to the hull body
    wheelConstraint->SetWorldPosition(wheelNode->GetPosition()); // Set constraint's both ends at wheel's location
    wheelConstraint->SetAxis(Vector3::UP); // Wheel rotates around its local Y-axis

    wheelConstraint->SetDisableCollision(true); // Let the wheel intersect the vehicle hull
}

void MurRobot::GetWheelComponents()
{
    leftBody_ = left_->GetComponent<RigidBody>();
    rightBody_ = right_->GetComponent<RigidBody>();
    upBody_ = up_->GetComponent<RigidBody>();
}

void MurRobot::WaterForce(const Vector3 &force) {
    hullBody_->ApplyForce(force);
}

Vector3 MurRobot::GetPosition() {
    return hullBody_->GetPosition();
}

float MurRobot::GetYaw() {
    return hullBody_->GetRotation().YawAngle();
}

float MurRobot::GetPitch() {
    return hullBody_->GetRotation().PitchAngle();
}

float MurRobot::GetRoll() {
    return hullBody_->GetRotation().RollAngle();
}

void MurRobot::SetThrusterPower(int A, int B, int C) {
    motorA_ = A / 100.0f;
    motorB_ = B / 100.0f;
    motorC_ = C / 100.0f;
}

