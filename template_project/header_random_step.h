#pragma once
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

class RandomStep
{
private:
    double block_vol = 20;
public:
    double size = 10;
    double height = 3;
    double thickness = 0.2;

    double block_size = 0.5;
    std::shared_ptr<ChBody> random_block[30][30];


    enum StepType {
        FLAT,
        RANDOM,
        SIN,
        STEP
    };
    //StepType step_type;


    RandomStep(ChSystemNSC& my_system, int step_type)
    {
        //---------------------
        // cleate field ofject
        //--------------------

        wall(my_system, size, height, thickness);
        set_block(my_system, step_type);
    }
    void wall(ChSystemNSC& my_system, double size, double height, double thickness = 0.5) {
        auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        ground_mat->SetFriction(1.0);

        auto wallL = chrono_types::make_shared<ChBodyEasyBox>(thickness, height, size, 30000, true, true, ground_mat);
        wallL->SetPos(ChVector<>(-size / 2 - thickness / 2, height / 2, 0));
        wallL->SetBodyFixed(true);
        my_system.Add(wallL);

        auto wallR = chrono_types::make_shared<ChBodyEasyBox>(thickness, height, size, 30000, true, true, ground_mat);
        wallR->SetPos(ChVector<>(+size / 2 + thickness / 2, height / 2, 0));
        wallR->SetBodyFixed(true);
        my_system.Add(wallR);

        auto wallF = chrono_types::make_shared<ChBodyEasyBox>(size, height, thickness, 30000, true, true, ground_mat);
        wallF->SetPos(ChVector<>(0, height / 2, -size / 2 - thickness / 2));
        wallF->SetBodyFixed(true);
        my_system.Add(wallF);

        auto wallB = chrono_types::make_shared<ChBodyEasyBox>(size, height, thickness, 30000, true, true, ground_mat);
        wallB->SetPos(ChVector<>(0, height / 2, size / 2 + thickness / 2));
        wallB->SetBodyFixed(true);
        my_system.Add(wallB);
    }
    void set_block(ChSystemNSC& my_system, int step_type) {
        auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        ground_mat->SetFriction(1.0);
        
        switch (step_type) {
        case FLAT:
            for (int i = 0; i < size / block_size; i++) {
                for (int j = 0; j < size / block_size; j++) {
                    double random_height = 1;
                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, random_height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + block_size / 2, random_height / 2, -size / 2 + j * block_size + block_size / 2));
                    random_block[i][j]->SetBodyFixed(true);
                    //random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_borders.png")));
                    auto color_truss = chrono_types::make_shared<ChColorAsset>();
                    color_truss->SetColor(ChColor(0.2f, 0.2f, 0.2f));
                    random_block[i][j]->AddAsset(color_truss);
                    random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_borders.png")));
                    my_system.Add(random_block[i][j]);
                }
            }
            break;

        case RANDOM:
            for (int i = 0; i < size / block_size; i++) {
                for (int j = 0; j < size / block_size; j++) {
                    double random_height = ChRandom() * 0.5;
                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, random_height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + block_size / 2, random_height / 2, -size / 2 + j * block_size + block_size / 2));
                    random_block[i][j]->SetBodyFixed(true);
                    random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_borders.png")));
                    //random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
                    auto color_truss = chrono_types::make_shared<ChColorAsset>();
                    color_truss->SetColor(ChColor(0.2f, 0.2f, 0.2f));
                    random_block[i][j]->AddAsset(color_truss);
                    my_system.Add(random_block[i][j]);
                }
            }
            break;
        case SIN:
            for (int i = 0; i < size /block_size; i++) {
                for (int j = 0; j < size /block_size; j++) {
                    double height = (std::sin(CH_C_PI * i / (size /block_size))) + (std::cos(CH_C_PI * j / (size*0.5/block_size)) * 0.5) + 0.5;
                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + block_size/2, height / 2, -size / 2 + j * block_size + block_size / 2));
                    random_block[i][j]->SetBodyFixed(true);
                    random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_borders.png")));
                    //random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
                    my_system.Add(random_block[i][j]);
                }
            }
            break;
        case STEP:
            for (int i = 0; i < size / block_size; i++) {
                for (int j = 0; j < size /block_size; j++) {
                    double height = 0;
                    if (i < 5) {
                        height = (j + 1) / (size/block_size);
                    }
                    else if (i < 10) {
                        height = 1 - (j / (size / block_size));
                    }
                    else if (i < 15) {
                        height = (j + 1) / (size / block_size);
                    }
                    else {
                        height = 1 - (j / (size / block_size));
                    }

                    random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, height, block_size, 30000, true, true, ground_mat);
                    random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + block_size / 2, height / 2, -size / 2 + j * block_size + block_size / 2));
                    random_block[i][j]->SetBodyFixed(true);
                    auto color_truss = chrono_types::make_shared<ChColorAsset>();
                    color_truss->SetColor(ChColor(0.2f, 0.2f, 0.2f));
                    random_block[i][j]->AddAsset(color_truss);
                    random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_borders.png")));
                    //random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
                    my_system.Add(random_block[i][j]);
                }
            }
            break;

        }
    }

};
