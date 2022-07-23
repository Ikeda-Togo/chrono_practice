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
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrMeshTools.h"

//#include "random_step.h"


// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

//random step type 
enum StepType {
    RANDOM,
    SIN,
    STEP
};

void CreateStatorRotor(std::shared_ptr<ChBody>& mstator,
    std::shared_ptr<ChBody>& mrotor,
    std::shared_ptr<ChMaterialSurface> material,
    ChSystem& msystem,
    const ChVector<>& mpos) {
    //mstator = chrono_types::make_shared<ChBodyEasyCylinder>(0.5, 0.1, 1000, material, collision_type);
    mstator = chrono_types::make_shared<ChBodyEasyBox>(0.3, 0.05, 0.3, 3000, material, collision_type);
    mstator->SetPos(mpos + ChVector<>(0, 0, -0.2));
    mstator->SetCollide(false);
    mstator->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    mstator->SetBodyFixed(true);
    msystem.Add(mstator);

    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("bluewhite.png"));
    mstator->AddAsset(texture);

    mrotor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 0.1, 10000, material, collision_type);
    mrotor->SetPos(mstator->GetPos() + ChVector<>(-1, 0, -0.1));
    msystem.Add(mrotor);

    auto mcolor = chrono_types::make_shared<ChColorAsset>(0.6f, 0.6f, 0.0f);
    mrotor->AddAsset(mcolor);
}

class RandomStep
{
private:
    double block_vol = 20;
public:
    double size = 10;
    double height = 1;
    double thickness = 0.2;
    
    double block_size = 0.5;
    std::shared_ptr<ChBody> random_block[30][30];

    //enum StepType {
    //    RANDOM,
    //    SIN,
    //    STEP
    //};
    //StepType step_type;


    RandomStep(ChSystemNSC& my_system ,int step_type)
    {
        //---------------------
        // cleate field ofject
        //--------------------

        wall(my_system,size, height, thickness);
        set_block(my_system,step_type);
    }
    void wall(ChSystemNSC& my_system,double size, double height,double thickness=0.5) {
        auto wallL = chrono_types::make_shared<ChBodyEasyBox>(thickness, height, size, 30000, true, true, nullptr);
        wallL->SetPos(ChVector<>(-size/2-thickness/2, height/2, 0));
        wallL->SetBodyFixed(true);
        my_system.Add(wallL);

        auto wallR = chrono_types::make_shared<ChBodyEasyBox>(thickness, height, size, 30000, true, true, nullptr);
        wallR->SetPos(ChVector<>(+size / 2 + thickness / 2, height / 2, 0));
        wallR->SetBodyFixed(true);
        my_system.Add(wallR);

        auto wallF = chrono_types::make_shared<ChBodyEasyBox>(size, height, thickness, 30000, true, true, nullptr);
        wallF->SetPos(ChVector<>(0, height / 2, -size / 2 - thickness / 2));
        wallF->SetBodyFixed(true);
        my_system.Add(wallF);

        auto wallB = chrono_types::make_shared<ChBodyEasyBox>(size, height, thickness, 30000, true, true, nullptr);
        wallB->SetPos(ChVector<>(0, height / 2, size / 2 + thickness / 2));
        wallB->SetBodyFixed(true);
        my_system.Add(wallB);
    }
    void set_block(ChSystemNSC& my_system,int step_type){
        auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        ground_mat->SetFriction(1.0);
        switch (step_type) {
        case RANDOM:
            for (int i = 0; i < size * 2; i++) {
                for (int j = 0; j < size * 2; j++) {
                    double random_height = ChRandom() * 0.5;
                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, random_height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + 0.25, random_height / 2, -size / 2 + j * block_size + 0.25));
                    random_block[i][j]->SetBodyFixed(true);
                    random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
                    my_system.Add(random_block[i][j]);
                }
            }
            break;
        case SIN:
            for (int i = 0; i < size*2; i++) {
                for (int j = 0; j < size * 2; j++) {
                    double height = (std::sin(CH_C_PI * i / (size * 2)))  +  (std::cos(CH_C_PI * j / (size * 1))*0.5)+0.5;
                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + 0.25, height / 2, -size / 2 + j * block_size + 0.25));
                    random_block[i][j]->SetBodyFixed(true);
                    random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
                    my_system.Add(random_block[i][j]);
                }
            }
            break;
        case STEP:
            for (int i = 0; i < size*2; i++) {
                for (int j = 0; j < size * 2; j++) {
                    double height = 0;
                    if (i < size/2) {
                        height = (j+1) / (size * 2);
                    }
                    else if (i<size) {
                        height = 1 - (j / (size * 2));
                    }
                    else if (i < size*3/2) {
                        height = (j+1) / (size * 2);
                    }
                    else {
                        height = 1 - (j / (size * 2));
                    }
                   
                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + 0.25, height / 2, -size / 2 + j * block_size + 0.25));
                    random_block[i][j]->SetBodyFixed(true);
                    auto color_truss = chrono_types::make_shared<ChColorAsset>();
                    color_truss->SetColor(ChColor(0.2f, 0.2f, 0.2f));
                    random_block[i][j]->AddAsset(color_truss);
                    //random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
                    my_system.Add(random_block[i][j]);
                }
            }
            break;

        }
    }

};


//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    GetLog() << GetChronoDataFile("models/bulldozer/wheel_view.obj").c_str() << "\n";

    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;
    my_system.SetCollisionSystemType(collision_type);
    my_system.Set_G_acc({ 0, -9.81, 0 });

    // Contact material shared among all objects
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    double angle = 0.0;

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Modeling a simplified   tank", core::dimension2d<u32>(1600, 1200));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(vector3df(0.f, 100.f, 0.f));
    application.AddTypicalCamera(core::vector3df(-6, 5, -6), core::vector3df(0, 0, 0));

    // 2- Create the rigid bodies of the simpified tank suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(1.0);

    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(30, 2, 30, 1000, true, true, ground_mat);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->SetBodyFixed(true);
    my_ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/blue.png")));
    my_system.AddBody(my_ground);

    //// ..some obstacles on the ground:
    //auto obst_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    //for (int i = 0; i < 30; i++) {
    //    auto my_obstacle = chrono_types::make_shared<ChBodyEasyBox>(
    //        0.6 * (1 - 0.4 * ChRandom()), 0.08, 0.3 * (1 - 0.4 * ChRandom()), 1000, true, true, ground_mat);
    //    my_obstacle->SetMass(1);
    //    //my_obstacle->SetPos(ChVector<>(-6 + 6 * ChRandom(), 2 + 1 * ChRandom(), 6 * ChRandom()));
    //    my_obstacle->SetPos(ChVector<>(-6 + 6 * ChRandom(), 2, 6 * ChRandom()));
    //    my_obstacle->SetRot(Q_from_AngAxis(ChRandom() * CH_C_PI, VECT_Y));
    //    my_system.AddBody(my_obstacle);
    //}

    StepType step_type=STEP;
    RandomStep* myrandomstep = new RandomStep(my_system ,step_type);


    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    //
    // SETTINGS
    //

    my_system.SetSolverType(ChSolver::Type::PSOR);
    my_system.SetSolverMaxIterations(100);  // the higher, the easier to keep the constraints satisfied.

    //
    // Simulation loop
    //
    //application.SetTimestep(1);
    application.SetTimestep(0.03);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();



        // .. draw also a grid (rotated so that it's horizontal)
        tools::drawGrid(application.GetVideoDriver(), 1, 1, 30, 30,
            ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
            video::SColor(255, 60, 60, 60), true);


        // HERE CHRONO INTEGRATION IS PERFORMED:
        application.DoStep();

        application.EndScene();
    }

    return 0;
}
