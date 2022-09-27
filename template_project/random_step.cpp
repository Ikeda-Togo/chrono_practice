//#include "header_random_step.h"
//
//
//
//
//RandomStep::RandomStep(ChSystemNSC& my_system, int step_type)
//{
//    //---------------------
//    // cleate field ofject
//    //--------------------
//    double size = 10;
//    double height = 1;
//    double thickness = 0.2;
//
//    double block_size = 0.5;
//
//    //wall(my_system, size, height, thickness);
//    //set_block(my_system,step_type, size, block_size);
//};
//
//void RandomStep::wall(ChSystemNSC& my_system, double size, double height, double thickness = 0.5) {
//    auto wallL = chrono_types::make_shared<ChBodyEasyBox>(thickness, height, size, 30000, true, true, nullptr);
//    wallL->SetPos(ChVector<>(-size / 2 - thickness / 2, height / 2, 0));
//    wallL->SetBodyFixed(true);
//    my_system.Add(wallL);
//
//    auto wallR = chrono_types::make_shared<ChBodyEasyBox>(thickness, height, size, 30000, true, true, nullptr);
//    wallR->SetPos(ChVector<>(+size / 2 + thickness / 2, height / 2, 0));
//    wallR->SetBodyFixed(true);
//    my_system.Add(wallR);
//
//    auto wallF = chrono_types::make_shared<ChBodyEasyBox>(size, height, thickness, 30000, true, true, nullptr);
//    wallF->SetPos(ChVector<>(0, height / 2, -size / 2 - thickness / 2));
//    wallF->SetBodyFixed(true);
//    my_system.Add(wallF);
//
//    auto wallB = chrono_types::make_shared<ChBodyEasyBox>(size, height, thickness, 30000, true, true, nullptr);
//    wallB->SetPos(ChVector<>(0, height / 2, size / 2 + thickness / 2));
//    wallB->SetBodyFixed(true);
//    my_system.Add(wallB);
//};
//
//void RandomStep::set_block(ChSystemNSC& my_system, int step_type, double size, double block_size) {
//    enum StepType {
//        RANDOM,
//        SIN,
//        STEP,
//    };
//    std::shared_ptr<ChBody> random_block[30][30];
//
//    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
//    ground_mat->SetFriction(1.0);
//    switch (step_type) {
//    case RANDOM:
//        for (int i = 0; i < size * 2; i++) {
//            for (int j = 0; j < size * 2; j++) {
//                double random_height = ChRandom() * 0.5;
//                random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, random_height, block_size, 30000, true, true, ground_mat);
//                random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + 0.25, random_height / 2, -size / 2 + j * block_size + 0.25));
//                random_block[i][j]->SetBodyFixed(true);
//                random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
//                my_system.Add(random_block[i][j]);
//            }
//        }
//        break;
//    case SIN:
//        for (int i = 0; i < size * 2; i++) {
//            for (int j = 0; j < size * 2; j++) {
//                double height = (std::sin(CH_C_PI * i / (size * 2))) + (std::cos(CH_C_PI * j / (size * 1)) * 0.5) + 0.5;
//                random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, height, block_size, 30000, true, true, ground_mat);
//                random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + 0.25, height / 2, -size / 2 + j * block_size + 0.25));
//                random_block[i][j]->SetBodyFixed(true);
//                random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
//                my_system.Add(random_block[i][j]);
//            }
//        }
//        break;
//    case STEP:
//        for (int i = 0; i < size * 2; i++) {
//            for (int j = 0; j < size * 2; j++) {
//                double height = 0;
//                if (i < size / 2) {
//                    height = (j + 1) / (size * 2);
//                }
//                else if (i < size) {
//                    height = 1 - (j / (size * 2));
//                }
//                else if (i < size * 3 / 2) {
//                    height = (j + 1) / (size * 2);
//                }
//                else {
//                    height = 1 - (j / (size * 2));
//                }
//
//                random_block[i][j] = chrono_types::make_shared<ChBodyEasyBox>(block_size, height, block_size, 30000, true, true, ground_mat);
//                random_block[i][j]->SetPos(ChVector<>(-size / 2 + i * block_size + 0.25, height / 2, -size / 2 + j * block_size + 0.25));
//                random_block[i][j]->SetBodyFixed(true);
//                auto color_truss = chrono_types::make_shared<ChColorAsset>();
//                color_truss->SetColor(ChColor(0.2f, 0.2f, 0.2f));
//                random_block[i][j]->AddAsset(color_truss);
//                //random_block[i][j]->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/spheretexture.png")));
//                my_system.Add(random_block[i][j]);
//            }
//        }
//        break;
//
//    }
//};