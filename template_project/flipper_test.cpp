// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - modeling tracks with articulated shoes (as an example of complex model with
//   collisions and constraints)
// - using different meshes for collision and visualization
// - using clones of collision shapes
// - using SetFamilyMaskNoCollisionWithFamily, SetFamily etc. to avoid
//   collisions between different families of bodies.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

#include "header_random_step.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace chrono::sensor;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


//random step type 
enum StepType {
    FLAT,
    RANDOM,
    SIN,
    STEP
};

// -----------------------------------------------------------------------------
// IMU parameters
// -----------------------------------------------------------------------------
// Noise model attached to the sensor
enum IMUNoiseModel {
    NORMAL_DRIFT,  // gaussian drifting noise with noncorrelated equal distributions
    IMU_NONE       // no noise added
};
IMUNoiseModel imu_noise_type = IMU_NONE;

// IMU update rate in Hz
int imu_update_rate = 100;

// IMU lag (in seconds) between sensing and when data becomes accessible
float imu_lag = 0;

// IMU collection time (in seconds) of each sample
float imu_collection_time = 0;


collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;


// First of all, define a class for the 'tank' (that is, a set of
// bodies and links which are grouped within this class; so it is
// easier to manage data structures in this example).

//�N���[���̃��f������

class MySimpleTank {
public:
    // THE DATA

    double throttleL;        // actual value 0...1 of gas throttle (left).
    double throttleR;        // actual value 0...1 of gas throttle (right).
    double max_motor_speed;  // the max rotation speed of the motor [rads/s]

    // The parts making the tank, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    // .. truss:
    std::shared_ptr<ChBody> truss;
    // .. right front suspension:
    std::shared_ptr<ChBody> wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
    // .. left front suspension:
    std::shared_ptr<ChBody> wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
    // .. right back suspension:
    std::shared_ptr<ChBody> wheelRB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_motorRB;
    // .. left back suspension:
    std::shared_ptr<ChBody> wheelLB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_motorLB;

    // .. right front suspension:
    std::shared_ptr<ChBody> flipper_wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_flipper_revoluteRF;
    // .. left front suspension:
    std::shared_ptr<ChBody> flipper_wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_flipper_revoluteLF;
    // .. right back suspension:
    std::shared_ptr<ChBody> flipper_wheelRB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_flipper_motorRB;
    // .. left back suspension:
    std::shared_ptr<ChBody> flipper_wheelLB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_flipper_motorLB;

    // THE FUNCTIONS

    // Build and initialize the tank, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleTank(ChSystemNSC& my_system,        ///< the Chrono physical system
        ISceneManager* msceneManager,  ///< the Irrlicht scene manager for 3d shapes
        IVideoDriver* mdriver ,         ///< the Irrlicht video driver
        double mx,
        double my
    ) {
        throttleL = throttleR = 0;  // initially, gas throttle is 0.
        max_motor_speed = 10;

        //double my = 0.5;  // left back hub pos
        //double mx = 0;

        double shoelength = 0.2;
        double shoemass = 2;
        double radiustrack = 0.31;
        double wheeldiameter = 0.280 * 2;
        int nwrap = 6;
        int ntiles = 7;
        double rlwidth = 1.20;
        double passo = (ntiles + 1) * shoelength;


        ChVector<> cyl_displA(0, 0.075 + 0.02, 0);
        ChVector<> cyl_displB(0, -0.075 - 0.02, 0);
        double cyl_hthickness = 0.045;

        // --- The tank body ---

        auto load_truss = chrono_types::make_shared<ChBodyEasyMesh>(               //
            "C:/Users/syuug/Documents/GitHub/chrono_practice/obj/truss.obj",  // data file
            1000,                                                          // density
            true,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            chrono_types::make_shared<ChMaterialSurfaceNSC>(),                                                       // no need for contact material
            0.0);                                                            // mesh sweep sphere radius
        //my_system.AddBody(Lflipper);
        load_truss->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, rlwidth / 2));
        load_truss->SetMass(350);
        std::cout << load_truss->GetInertiaXX() << std::endl;

        truss = load_truss;
        
        auto color_truss = chrono_types::make_shared<ChColorAsset>();
        color_truss->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        truss->AddAsset(color_truss);

        my_system.Add(truss);

        //auto color_truss = chrono_types::make_shared<ChColorAsset>();
        //color_truss->SetColor(ChColor(1.0f, 0.2f, 0.2f));
        //truss->AddAsset(color_truss);


        // --- Contact material for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        wheel_mat->SetFriction(1.0);

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, my + radiustrack, 0));
        wheelRF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRF->SetMass(9.0);
        wheelRF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));

        wheelRF->GetCollisionModel()->ClearModel();
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelRF->GetCollisionModel()->BuildModel();
        wheelRF->SetCollide(true);

        auto color_wheel = chrono_types::make_shared<ChColorAsset>();
        color_wheel->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        wheelRF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss, ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, 0), QUNIT));
        my_system.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, my + radiustrack, rlwidth));
        wheelLF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelLF->AddAsset(color_wheel);

        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, truss,
            ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, rlwidth), QUNIT));
        my_system.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, my + radiustrack, 0));
        wheelRB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelRB->AddAsset(color_wheel);

        wheelRB->GetCollisionModel()->ClearModel();
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelRB->GetCollisionModel()->BuildModel();
        wheelRB->SetCollide(true);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, truss, ChFrame<>(ChVector<>(mx, my + radiustrack, 0), QUNIT));
        my_system.AddLink(link_motorRB);

        // --- Left Back suspension ---

        // ..the tank left-back wheel

        wheelLB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, my + radiustrack, rlwidth));
        wheelLB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelLB->AddAsset(color_wheel);

        wheelLB->GetCollisionModel()->ClearModel();
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelLB->GetCollisionModel()->BuildModel();
        wheelLB->SetCollide(true);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, truss, ChFrame<>(ChVector<>(mx, my + radiustrack, rlwidth), QUNIT));
        my_system.AddLink(link_motorLB);

        //--- TRACKS ---

        // Load a triangle mesh for collision

        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("models/bulldozer/shoe_collision.obj").c_str());

        auto trimesh = chrono_types::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh.get(), irmesh_shoe_collision->getMesh(0));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

        chrono::ChVector<> position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            mx = 0;
            mx += shoelength;

            double mz = 0;

            if (side == 0)
                mz = 0;
            else
                mz = rlwidth;

            position.Set(mx, my, mz);
            rotation = QUNIT;

            // Create sample body (with empty collision shape; later create the collision model by adding the
            // coll.shapes)
            auto firstBodyShoe = chrono_types::make_shared<ChBody>();
            my_system.Add(firstBodyShoe);
            firstBodyShoe->SetMass(shoemass);
            firstBodyShoe->SetPos(position);
            firstBodyShoe->SetRot(rotation);
            firstBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            auto shoe_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_mesh);
            shoe_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/bulldozer/shoe_view.obj").c_str());
            shoe_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_coll_mesh);
            shoe_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/bulldozer/shoe_collision.obj").c_str());
            shoe_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_coll_mesh->SetVisible(false);

            // Collision:
            firstBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            firstBodyShoe->GetCollisionModel()->SetEnvelope(0.010);    // distance of the outward "collision envelope"
            firstBodyShoe->GetCollisionModel()->ClearModel();
            firstBodyShoe->GetCollisionModel()->AddTriangleMesh(chrono_types::make_shared<ChMaterialSurfaceNSC>(),
                trimesh, false, false, mesh_displacement,
                ChMatrix33<>(1), 0.005);
            firstBodyShoe->GetCollisionModel()->BuildModel();  // Creates the collision model
            firstBodyShoe->SetCollide(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            firstBodyShoe->GetCollisionModel()->SetFamily(3);
            firstBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

            std::shared_ptr<ChBody> previous_rigidBodyShoe;
            previous_rigidBodyShoe = firstBodyShoe;

            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + shoelength + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_C_PI + (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            // close track
            ChVector<> linkpos = firstBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(firstBodyShoe, previous_rigidBodyShoe, ChCoordsys<>(linkpos, QUNIT));
            my_system.AddLink(link_revolute_shoeshoe);
        }
    }

    // Delete the tank object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleTank() {
        ChSystem* mysystem = truss->GetSystem();  // trick to get the system here

        mysystem->Remove(link_revoluteRF);
        mysystem->Remove(link_revoluteLF);
        mysystem->Remove(link_motorRB);
        mysystem->Remove(link_motorLB);
        mysystem->Remove(truss);
        mysystem->Remove(wheelRF);
        mysystem->Remove(wheelLF);
        mysystem->Remove(wheelRB);
        mysystem->Remove(wheelLB);
    }

    // Utility function to create quickly a track shoe connected to the previous one
    std::shared_ptr<ChBody> MakeShoe(
        std::shared_ptr<ChBody> previous_shoe,  // will be linked with this one with revolute joint
        std::shared_ptr<ChBody> template_shoe,  // collision geometry will be shared with this body
        ChVector<> position,                    // position
        ChQuaternion<> rotation,                // orientation
        ChSystemNSC& my_system,                 // the physical system
        chrono::ChVector<> joint_displacement   // position of shoe-shoe constraint, relative to COG.
    ) {
        auto rigidBodyShoe = std::shared_ptr<ChBody>(template_shoe.get()->Clone());
        rigidBodyShoe->SetPos(position);
        rigidBodyShoe->SetRot(rotation);

        auto color_shoe = chrono_types::make_shared<ChColorAsset>();
        color_shoe->SetColor(ChColor(0.0f, 0.2f, 0.0f));

        rigidBodyShoe->AddAsset(color_shoe);


        my_system.Add(rigidBodyShoe);

        rigidBodyShoe->GetCollisionModel()->ClearModel();
        rigidBodyShoe->GetCollisionModel()->AddCopyOfAnotherModel(template_shoe->GetCollisionModel().get());
        rigidBodyShoe->GetCollisionModel()->BuildModel();
        rigidBodyShoe->SetCollide(true);

        // Other settings are already copied from template_shoe, except for family and mask.
        // Avoid creation of contacts with neighbouring shoes:
        rigidBodyShoe->GetCollisionModel()->SetFamily(3);
        rigidBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

        // Create revolute constraint
        if (previous_shoe) {
            ChVector<> linkpos = rigidBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(rigidBodyShoe, previous_shoe, ChCoordsys<>(linkpos, QUNIT));
            my_system.AddLink(link_revolute_shoeshoe);
        }

        return rigidBodyShoe;
    }
};

class MySimpleFlipper {
public:
    // THE DATA

    double throttleL;        // actual value 0...1 of gas throttle (left).
    double throttleR;        // actual value 0...1 of gas throttle (right).
    double max_motor_speed;  // the max rotation speed of the motor [rads/s]

    // The parts making the tank, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    // .. truss:
    //std::shared_ptr<ChBody> truss;
    std::shared_ptr<ChBody> trussR;
    std::shared_ptr<ChBody> trussL;
    // .. right front suspension:
    std::shared_ptr<ChBody> wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
    // .. left front suspension:
    std::shared_ptr<ChBody> wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
    // .. right back suspension:
    std::shared_ptr<ChBody> wheelRB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_motorRB;
    // .. left back suspension:
    std::shared_ptr<ChBody> wheelLB;
    std::shared_ptr<ChLinkMotorRotationSpeed> link_motorLB;


    // THE FUNCTIONS

    // Build and initialize the tank, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleFlipper(ChSystemNSC& my_system,        ///< the Chrono physical system
        ISceneManager* msceneManager,  ///< the Irrlicht scene manager for 3d shapes
        IVideoDriver* mdriver,         ///< the Irrlicht video driver
        double mx,
        double my,
        double mz
    ) {
        throttleL = throttleR = 0;  // initially, gas throttle is 0.
        max_motor_speed = 10;

        //double my = 0.5;  // left back hub pos
        //double mx = 0;

        double shoelength = 0.2;
        double shoemass = 2;
        double radiustrack = 0.31;
        double wheeldiameter = 0.280 * 2;
        int nwrap = 6;
        int ntiles = 4;
        double rlwidth = 2.0;
        double passo = (ntiles + 1) * shoelength;
        printf("passo %lf\r\n", passo);

        ChVector<> cyl_displA(0, 0.075 + 0.02, 0);
        ChVector<> cyl_displB(0, -0.075 - 0.02, 0);
        double cyl_hthickness = 0.045;

        // --- The tank body ---

        trussR = std::make_shared<ChBodyEasyBox>(passo, 0.4, 0.01,  // x, y, z dimensions
            10000,       // density
            chrono_types::make_shared<ChMaterialSurfaceNSC>(),       // create visualization asset
            collision_type       // no collision geometry
            );
        trussR->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, mz + 0.2));
        trussR->SetMass(150);

        my_system.Add(trussR);


        trussL = std::make_shared<ChBodyEasyBox>(passo, 0.4, 0.01,  // x, y, z dimensions
            10000,       // density
            chrono_types::make_shared<ChMaterialSurfaceNSC>(),       // create visualization asset
            collision_type       // no collision geometry
            );
        trussL->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, mz + rlwidth -0.2));
        trussL->SetMass(150);

        my_system.Add(trussL);


        // --- Contact material for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        wheel_mat->SetFriction(1.0);

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, my + radiustrack, mz));
        wheelRF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRF->SetMass(9.0);
        wheelRF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));

        wheelRF->GetCollisionModel()->ClearModel();
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelRF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelRF->GetCollisionModel()->BuildModel();
        wheelRF->SetCollide(true);

        auto color_wheel = chrono_types::make_shared<ChColorAsset>();
        color_wheel->SetColor(ChColor(0.2f, 0.2f, 0.2f));
        wheelRF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, trussR, ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, mz), QUNIT));
        my_system.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, my + radiustrack, mz + rlwidth));
        wheelLF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelLF->AddAsset(color_wheel);

        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, trussL,
            ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, mz + rlwidth), QUNIT));
        my_system.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, my + radiustrack, mz));
        wheelRB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelRB->AddAsset(color_wheel);

        wheelRB->GetCollisionModel()->ClearModel();
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelRB->GetCollisionModel()->BuildModel();
        wheelRB->SetCollide(true);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, trussR, ChFrame<>(ChVector<>(mx, my + radiustrack, mz), QUNIT));
        my_system.AddLink(link_motorRB);

        // --- Left Back suspension ---

        // ..the tank left-back wheel

        wheelLB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, my + radiustrack, mz + rlwidth));
        wheelLB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        wheelLB->AddAsset(color_wheel);

        wheelLB->GetCollisionModel()->ClearModel();
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelLB->GetCollisionModel()->BuildModel();
        wheelLB->SetCollide(true);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, trussL, ChFrame<>(ChVector<>(mx, my + radiustrack, mz + rlwidth), QUNIT));
        my_system.AddLink(link_motorLB);

        //--- TRACKS ---

        // Load a triangle mesh for collision

        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("models/bulldozer/shoe_collision.obj").c_str());

        auto trimesh = chrono_types::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh.get(), irmesh_shoe_collision->getMesh(0));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

        chrono::ChVector<> position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            //mx = 0;
            mx += shoelength;

            if (side == 0)
                mz = mz;
            else
                mz = mz+rlwidth;

            position.Set(mx, my, mz);
            rotation = QUNIT;

            // Create sample body (with empty collision shape; later create the collision model by adding the
            // coll.shapes)
            auto firstBodyShoe = chrono_types::make_shared<ChBody>();
            my_system.Add(firstBodyShoe);
            firstBodyShoe->SetMass(shoemass);
            firstBodyShoe->SetPos(position);
            firstBodyShoe->SetRot(rotation);
            firstBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            auto shoe_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_mesh);
            shoe_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/bulldozer/shoe_view.obj").c_str());
            shoe_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_coll_mesh);
            shoe_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/bulldozer/shoe_collision.obj").c_str());
            shoe_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_coll_mesh->SetVisible(false);

            // Collision:
            firstBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            firstBodyShoe->GetCollisionModel()->SetEnvelope(0.010);    // distance of the outward "collision envelope"
            firstBodyShoe->GetCollisionModel()->ClearModel();
            firstBodyShoe->GetCollisionModel()->AddTriangleMesh(chrono_types::make_shared<ChMaterialSurfaceNSC>(),
                trimesh, false, false, mesh_displacement,
                ChMatrix33<>(1), 0.005);
            firstBodyShoe->GetCollisionModel()->BuildModel();  // Creates the collision model
            firstBodyShoe->SetCollide(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            firstBodyShoe->GetCollisionModel()->SetFamily(3);
            firstBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

            std::shared_ptr<ChBody> previous_rigidBodyShoe;
            previous_rigidBodyShoe = firstBodyShoe;

            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + shoelength + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_C_PI + (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                auto rigidBodyShoe =
                    MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                previous_rigidBodyShoe = rigidBodyShoe;
            }
            // close track
            ChVector<> linkpos = firstBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(firstBodyShoe, previous_rigidBodyShoe, ChCoordsys<>(linkpos, QUNIT));
            my_system.AddLink(link_revolute_shoeshoe);
        }
    }

    // Delete the tank object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleFlipper() {
        ChSystem* mysystem = trussR->GetSystem();  // trick to get the system here

        mysystem->Remove(link_revoluteRF);
        mysystem->Remove(link_revoluteLF);
        mysystem->Remove(link_motorRB);
        mysystem->Remove(link_motorLB);
        mysystem->Remove(trussR);
        mysystem->Remove(wheelRF);
        mysystem->Remove(wheelLF);
        mysystem->Remove(wheelRB);
        mysystem->Remove(wheelLB);
    }

    // Utility function to create quickly a track shoe connected to the previous one
    std::shared_ptr<ChBody> MakeShoe(
        std::shared_ptr<ChBody> previous_shoe,  // will be linked with this one with revolute joint
        std::shared_ptr<ChBody> template_shoe,  // collision geometry will be shared with this body
        ChVector<> position,                    // position
        ChQuaternion<> rotation,                // orientation
        ChSystemNSC& my_system,                 // the physical system
        chrono::ChVector<> joint_displacement   // position of shoe-shoe constraint, relative to COG.
    ) {
        auto rigidBodyShoe = std::shared_ptr<ChBody>(template_shoe.get()->Clone());
        rigidBodyShoe->SetPos(position);
        rigidBodyShoe->SetRot(rotation);

        auto color_shoe = chrono_types::make_shared<ChColorAsset>();
        color_shoe->SetColor(ChColor(0.0f, 0.2f, 0.0f));

        rigidBodyShoe->AddAsset(color_shoe);


        my_system.Add(rigidBodyShoe);

        rigidBodyShoe->GetCollisionModel()->ClearModel();
        rigidBodyShoe->GetCollisionModel()->AddCopyOfAnotherModel(template_shoe->GetCollisionModel().get());
        rigidBodyShoe->GetCollisionModel()->BuildModel();
        rigidBodyShoe->SetCollide(true);

        // Other settings are already copied from template_shoe, except for family and mask.
        // Avoid creation of contacts with neighbouring shoes:
        rigidBodyShoe->GetCollisionModel()->SetFamily(3);
        rigidBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

        // Create revolute constraint
        if (previous_shoe) {
            ChVector<> linkpos = rigidBodyShoe->Point_Body2World(joint_displacement);
            auto link_revolute_shoeshoe = chrono_types::make_shared<ChLinkLockRevolute>();
            link_revolute_shoeshoe->Initialize(rigidBodyShoe, previous_shoe, ChCoordsys<>(linkpos, QUNIT));
            my_system.AddLink(link_revolute_shoeshoe);
        }

        return rigidBodyShoe;
    }
};

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
public:
    s32 pos_speed = 50, pos_handle = 50;
    MyEventReceiver(ChIrrAppInterface* myapp, MySimpleTank* atank, MySimpleFlipper* aflipper, double* angleL, double* angleR) {
        // store pointer application
        application = myapp;
        // store pointer to other stuff
        mtank = atank;
        mflipper = aflipper;
        mangleL = angleL;
        mangleR = angleR;


        // ..add a GUI slider to control throttle left via mouse
        scrollbar_throttleL =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 20, 650, 35), 0, 101);
        scrollbar_throttleL->setMax(100);
        scrollbar_throttleL->setPos(50);
        text_throttleL =
            application->GetIGUIEnvironment()->addStaticText(L"Speed ", rect<s32>(650, 20, 750, 35), false);

        // ..add a GUI slider to control gas throttle right via mouse
        scrollbar_throttleR =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 45, 650, 60), 0, 102);
        scrollbar_throttleR->setMax(100);
        scrollbar_throttleR->setPos(50);
        text_throttleR =
            application->GetIGUIEnvironment()->addStaticText(L"handle", rect<s32>(650, 45, 750, 60), false);

        // ..add a GUI slider to control throttle left via mouse
        scrollbar_flipperL =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 70, 650, 85), 0, 103);
        scrollbar_flipperL->setMax(100);
        scrollbar_flipperL->setPos(50);
        text_flipperL =
            application->GetIGUIEnvironment()->addStaticText(L"FlipperL ", rect<s32>(650, 70, 750, 85), false);

        // ..add a GUI slider to control throttle left via mouse
        scrollbar_flipperR =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 95, 650, 110), 0, 104);
        scrollbar_flipperR->setMax(100);
        scrollbar_flipperR->setPos(50);
        text_flipperR =
            application->GetIGUIEnvironment()->addStaticText(L"FlipperR ", rect<s32>(650, 95, 750, 110), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();


            switch (event.GUIEvent.EventType) {
            case EGET_SCROLL_BAR_CHANGED:
                if (id == 101) {  // id of 'throttleL' slider..
                    pos_speed = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    double newthrottleL = ((double)(pos_speed)-50) / 50.0;
                    double newthrottleR = newthrottleL;
                    newthrottleL = newthrottleL - (((double)(pos_handle)-50) / 50.0);
                    newthrottleR = newthrottleR + (((double)(pos_handle)-50) / 50.0);

                    this->mtank->throttleL = newthrottleL;
                    this->mflipper->throttleL = newthrottleL;
                    auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleL * 6);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mflipper->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleL * 6);

                    this->mtank->throttleR = newthrottleR;
                    this->mflipper->throttleR = newthrottleR;

                    mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleR * 6);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mflipper->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleR * 6);


                    return true;
                }
                if (id == 102) {  // id of 'throttleR' slider..
                    pos_handle = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    double newthrottleL = ((double)(pos_speed)-50) / 50.0;
                    double newthrottleR = newthrottleL;
                    newthrottleL = newthrottleL - (((double)(pos_handle)-50) / 50.0);
                    newthrottleR = newthrottleR + (((double)(pos_handle)-50) / 50.0);

                    this->mtank->throttleL = newthrottleL;
                    this->mflipper->throttleL = newthrottleL;
                    auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleL * 6);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mflipper->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleL * 6);

                    this->mtank->throttleR = newthrottleR;
                    this->mflipper->throttleR = newthrottleR;

                    mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleR * 6);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mflipper->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst(newthrottleR * 6);

                    return true;
                }
                if (id == 103) {  // id of 'throttleL' slider..
                    s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    *mangleL = CH_C_PI * (pos - 50) / 100;

                    return true;
                }
                if (id == 104) {  // id of 'throttleL' slider..
                    s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    *mangleR = CH_C_PI * (pos - 50) / 100;

                    return true;
                }
                break;
            default:
                break;
            }
        }

        return false;
    }

private:
    ChIrrAppInterface* application;
    MySimpleTank* mtank;
    MySimpleFlipper* mflipper;
    double* mangleL;
    double* mangleR;

    IGUIStaticText* text_throttleL;
    IGUIScrollBar* scrollbar_throttleL;
    IGUIStaticText* text_throttleR;
    IGUIScrollBar* scrollbar_throttleR;
    IGUIStaticText* text_flipperL;
    IGUIScrollBar* scrollbar_flipperL;
    IGUIStaticText* text_flipperR;
    IGUIScrollBar* scrollbar_flipperR;
};


//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;
    my_system.SetCollisionSystemType(collision_type);
    //my_system.Set_G_acc({ 0, -9.81, 0 });

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Modeling a simplified   tank", core::dimension2d<u32>(1200, 800));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-3, 5, -3), core::vector3df(0, 0, 0));

    // 2- Create the rigid bodies of the simpified tank suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(1.0);

    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(60, 2, 60, 1000, true, true, ground_mat);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->SetBodyFixed(true);
    my_ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/blue.png")));
    my_system.AddBody(my_ground);

    my_system.Set_G_acc({ 0, -9.81, 0 });

    //// ..some obstacles on the ground:
    //auto obst_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    //for (int i = 0; i < 30; i++) {
    //    auto my_obstacle = chrono_types::make_shared<ChBodyEasyBox>(
    //        0.6 * (1 - 0.4 * ChRandom()), 0.08, 0.3 * (1 - 0.4 * ChRandom()), 1000, true, true, obst_mat);
    //    my_obstacle->SetMass(1);
    //    //my_obstacle->SetPos(ChVector<>(-6 + 6 * ChRandom(), 2 + 1 * ChRandom(), 6 * ChRandom()));
    //    my_obstacle->SetPos(ChVector<>(-6 + 6 * ChRandom(), 2, 6 * ChRandom()));
    //    my_obstacle->SetRot(Q_from_AngAxis(ChRandom() * CH_C_PI, VECT_Y));
    //    my_system.AddBody(my_obstacle);
    //}

    //------------------------------------------
    // create robot model
    //------------------------------------------
    //
    // ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)

    double model_height = 3;
    MySimpleTank* mytank = new MySimpleTank(my_system, application.GetSceneManager(), application.GetVideoDriver(), 0, model_height);
    ChVector<> center_pos = mytank->wheelLB->GetPos() - mytank->wheelLF->GetPos();
    MySimpleFlipper* myflipper = new MySimpleFlipper(
        my_system,
        application.GetSceneManager(),
        application.GetVideoDriver(),
        -1,
         model_height, 
        -0.4
    );

    // Create the motor
    auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(myflipper->trussL,                // body A (slave)
        mytank->truss,               // body B (master)
        ChFrame<>(mytank->wheelLB->GetPos())  // motor frame, in abs. coords
    );
    my_system.Add(rotmotor1);

    auto motor_funL = chrono_types::make_shared<ChFunction_Setpoint>();
    rotmotor1->SetAngleFunction(motor_funL);


    // Create the motor
    auto rotmotor2= chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor2->Initialize(myflipper->trussR,                // body A (slave)
        mytank->truss,               // body B (master)
        ChFrame<>(mytank->wheelRB->GetPos())  // motor frame, in abs. coords
    );
    my_system.Add(rotmotor2);

    auto motor_funR = chrono_types::make_shared<ChFunction_Setpoint>();
    rotmotor2->SetAngleFunction(motor_funR);

    auto tailmesh = chrono_types::make_shared<ChBodyEasyMesh>(               //
        "C:/Users/syuug/Documents/GitHub/chrono_practice/obj/tail.obj",  // data file
        10000,                                                          // density
        false,                                                         // do not compute mass and inertia
        true,                                                          // visualization?
        false,                                                         // collision?
        nullptr,               // no need for contact material
        0);                                                            // mesh sweep sphere radius
    //tailmesh->SetBodyFixed(true);
    my_system.Add(tailmesh);
    tailmesh->SetRot(Q_from_AngAxis(0, VECT_X));
    tailmesh->SetPos(mytank->truss->GetPos() + ChVector<>(1.05, 0.433, 0));
    tailmesh->SetMass(100);

    auto color_tail = chrono_types::make_shared<ChColorAsset>();
    color_tail->SetColor(ChColor(0.2f, 0.2f, 0.2f));
    tailmesh->AddAsset(color_tail);

    auto link_tail = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
    link_tail->Initialize(tailmesh, mytank->truss,
        ChCoordsys<>(tailmesh->GetPos(), QUNIT));
    link_tail->Lock(true);
    my_system.AddLink(link_tail);


    auto imumesh = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.1, 0.5, 30000, true, false, chrono_types::make_shared<ChMaterialSurfaceNSC>());
    imumesh->SetPos(mytank->truss->GetPos()+ChVector<>(0, 0.5, 0));
    imumesh->SetMass(10);
    std::cout << "mass is" << imumesh->GetMass() << std::endl;
    imumesh->SetCollide(true);
    my_system.Add(imumesh);

    auto link_imu = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
    link_imu->Initialize(imumesh, mytank->truss,
        ChCoordsys<>(mytank->truss->GetPos(), QUNIT));
    link_imu->Lock(true);
    my_system.AddLink(link_imu);

    //auto pole = chrono_types::make_shared<ChBodyEasyBox>(0.1,0.1,3, 30000, true, false, nullptr);
    //pole->SetPos(ChVector<>(0,5,0));
    //pole->SetBodyFixed(true);
    //my_system.Add(pole);



    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&my_system);

    // ---------------------------------------------
    // Create a IMU and add it to the sensor manager
    // ---------------------------------------------
    // Create the imu noise model
    std::shared_ptr<ChNoiseModel> acc_noise_model;
    std::shared_ptr<ChNoiseModel> gyro_noise_model;
    std::shared_ptr<ChNoiseModel> mag_noise_model;

    acc_noise_model = chrono_types::make_shared<ChNoiseNone>();
    gyro_noise_model = chrono_types::make_shared<ChNoiseNone>();
    mag_noise_model = chrono_types::make_shared<ChNoiseNone>();

    auto imu_offset_pose = chrono::ChFrame<double>({ 0, 0, 0 }, Q_from_AngAxis(0, { 1, 0, 0 }));
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(imumesh,    // body to which the IMU is attached
        imu_update_rate,   // update rate
        imu_offset_pose,   // offset pose from body
        acc_noise_model);  // IMU noise model
    acc->SetName("IMU - Accelerometer");
    acc->SetLag(imu_lag);
    acc->SetCollectionWindow(imu_collection_time);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());  // Add a filter to access the imu data
    manager->AddSensor(acc);                                            // Add the IMU sensor to the sensor manager

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(imumesh,     // body to which the IMU is attached
        imu_update_rate,    // update rate
        imu_offset_pose,    // offset pose from body
        gyro_noise_model);  // IMU noise model
    gyro->SetName("IMU - Accelerometer");
    gyro->SetLag(imu_lag);
    gyro->SetCollectionWindow(imu_collection_time);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());  // Add a filter to access the imu data
    manager->AddSensor(gyro);                                           // Add the IMU sensor to the sensor manager

    UserAccelBufferPtr bufferAcc;
    UserGyroBufferPtr bufferGyro;

    double acc_data[3];
    acc_data[0] = 0;
    acc_data[1] = 1;
    acc_data[2] = 2;

    std::wstringstream ss[3];
    IGUIStaticText* acc_data_text[3];

    ss[0] << L"x acc :";
    ss[1] << L"y acc :";
    ss[2] << L"z acc :";
    
    for (int i = 0; i < 3; i++) {
        ss[i] << acc_data[i];

        acc_data_text[i] =
            application.GetIGUIEnvironment()->addStaticText(ss[i].str().c_str(), rect<s32>(300, 20 + (i * 25), 750, 35 + i * 25), false);
  
    }


    int imu_last_launch = 0;

    // -----------------
    // Initialize output
    // -----------------
    
    // Output directories
    const std::string out_dir = "SENSOR_OUTPUT/";

    std::string imu_file = out_dir + "imu/";

    if (!filesystem::create_directory(filesystem::path(imu_file))) {
        std::cout << "Error creating directory " << imu_file << std::endl;
        return 1;
    }

    imu_file += "pendulum_leg_1.csv";
    utils::CSV_writer imu_csv(",");


    
    std::cout << "center pos is" << mytank->truss->GetPos() << std::endl;
    std::cout << "center flipper is" << (myflipper->trussL->GetPos().z() + myflipper->trussR->GetPos().z())/2<< std::endl;

    //---------------------
    // cleate field ofject
    //--------------------

    StepType step_type = FLAT ;
    RandomStep* myrandomstep = new RandomStep(my_system, step_type);

    //auto obj_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    //obj_mat->SetFriction(1.0);
    //auto box = chrono_types::make_shared<ChBodyEasyBox>(0.1, 3, 0.1, 30000, true, true, obj_mat);
    //box->SetPos(mytank->truss->GetPos());
    //box->SetCollide(false);
    //box->SetBodyFixed(true);
    //my_system.Add(box);


    //auto slope = chrono_types::make_shared<ChBodyEasyMesh>(               //
    //    "C:/Users/syuug/Documents/GitHub/chrono_practice/obj/step.obj",  // data file
    //    1000,                                                          // density
    //    true,                                                         // do not compute mass and inertia
    //    true,                                                          // visualization?
    //    true,                                                         // collision?
    //    chrono_types::make_shared<ChMaterialSurfaceNSC>(),                                                       // no need for contact material
    //    0.005);                                                            // mesh sweep sphere radius
    //slope->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
    //my_system.AddBody(slope);
    //slope->SetBodyFixed(true);
    //slope->SetPos(ChVector<>(-10, 1, 0));

    //
    //---create flipper arm-------
    //
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    double angleL = 0;
    double angleR = 0;

    // Create a ChFunction to be used for the ChLinkMotorRotationAngle
    auto msineangle = chrono_types::make_shared<ChFunction_Const>(0);       // phase [rad]




    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object.
    MyEventReceiver receiver(&application, mytank, myflipper, &angleL, &angleR);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    //
    // SETTINGS
    //

    my_system.SetSolverType(ChSolver::Type::PSOR);
    my_system.SetSolverMaxIterations(100);  // the higher, the easier to keep the constraints satisfied.

    //
    // Simulation loop
    //
    //application.SetTimestep(1);
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);
    
    ChVector<> now_pos(mytank->truss->GetPos());

    auto color_nowpos = chrono_types::make_shared<ChColorAsset>();
    color_nowpos->SetColor(ChColor(0.2f, 1.0f, 0.2f));

    auto color_red = chrono_types::make_shared<ChColorAsset>();
    color_red->SetColor(ChColor(1.0f, 0.2f, 0.2f));
    bool asset_change[30][30] = { false };

    while (application.GetDevice()->run()) {
        double t = my_system.GetChTime();
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        //application.GetActiveCamera()->setTarget(core::vector3dfCH(mytank->truss->GetPos()));


        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. draw also a grid (rotated so that it's horizontal)
        tools::drawGrid(application.GetVideoDriver(), 2, 2, 30, 30,
            ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
            video::SColor(255, 60, 60, 60), true);

        //std::cout << angle << std::endl;
        //msineangle->Set_yconst(angle);
        motor_funL->SetSetpoint(angleL, 0.5);
        motor_funR->SetSetpoint(angleR, 0.5);

        bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
        if (bufferAcc->Buffer && bufferGyro->Buffer) {
            // Save the imu data to file
            acc_data[0] = bufferAcc->Buffer[0].X;
            acc_data[1] = bufferAcc->Buffer[0].Y;
            acc_data[2] = bufferAcc->Buffer[0].Z;
            GyroData gyro_data = bufferGyro->Buffer[0];

            for (int i = 0; i < 3; i++) {
                ss[i].str(L"");
                if (i == 0) ss[i] << L"X acc:";
                else if(i==1)ss[i] << L"Y acc:";
                else ss[i] << L"Z acc:";

                ss[i] << acc_data[i];
                acc_data_text[i]->setText(ss[i].str().c_str());

            }

            //imu_csv << std::fixed << std::setprecision(6);
            imu_csv << t;
            imu_csv << acc_data[0];
            imu_csv << acc_data[1];
            imu_csv << acc_data[2];
            imu_csv << gyro_data.Roll;
            imu_csv << gyro_data.Pitch;
            imu_csv << gyro_data.Yaw;
            //imu_csv << mag_data.X;
            //imu_csv << mag_data.Y;
            //imu_csv << mag_data.Z;
            imu_csv << std::endl;
            imu_last_launch = bufferGyro->LaunchedCount;
            //printf("%0.4f %0.4f %0.4f \n", acc_data.X, acc_data.Y, acc_data.Z);
        }
        manager->Update();

        //���{�b�g��̃I�u�W�F��΂ɂ���
        now_pos =mytank->truss->GetPos();

        // if (acc_data.Y > 50) {
        //    myrandomstep->random_block[int(floor(now_pos.x())) + 10][int(floor(now_pos.z())) + 10]->AddAsset(color_red);
        //    application.AssetBindAll();
        //    application.AssetUpdateAll();
        //    asset_change[int(floor(now_pos.x())) + 10][int(floor(now_pos.z())) + 10] = true;
        // }
        
        // if (!asset_change[int(floor(now_pos.x())) + 10][int(floor(now_pos.z())) + 10]) {
        //    myrandomstep->random_block[int(floor(now_pos.x())) + 10][int(floor(now_pos.z())) + 10]->AddAsset(color_nowpos);
        //    application.AssetBindAll();
        //    application.AssetUpdateAll();
        //    asset_change[int(floor(now_pos.x())) + 10][int(floor(now_pos.z())) + 10] = true;
        //    //GetLog() << myrandomstep->random_block[int(now_pos.x()) + 10][int(now_pos.z()) + 9]->GetAssets() << "\n";
        //    //std::cout << "x is:" << int(now_pos.x()) + 10 << "z is:" << int(now_pos.z()) + 9 << std::endl;
        // }

        // HERE CHRONO INTEGRATION IS PERFORMED:
        application.DoStep();

        application.EndScene();
    }
    imu_csv.write_to_file(imu_file);

    if (mytank)
        delete mytank;

    return 0;
}
