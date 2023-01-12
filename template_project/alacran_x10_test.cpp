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
//#include "header_alacran.h"
#include "header_alacran_x10.h"

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

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

//random step type 
enum StepType {
    FLAT,
    RANDOM,
    SIN,
    STEP
};


class MyEventReceiver : public IEventReceiver {
public:
    s32 pos_speed = 50, pos_handle = 50;
    MyEventReceiver(ChIrrAppInterface* myapp, 
        MySimpleTank* atank, 
        MySimpleFlipper* aflipper, 
        MySimpleBackunit* abackunit, 
        double* angleL, 
        double* angleR,
        double* TL_angle1,
        double* TL_angle2) {
        // store pointer application
        application = myapp;
        // store pointer to other stuff
        mtank = atank;
        mflipper = aflipper;
        mbackunit = abackunit;
        mangleL = angleL;
        mangleR = angleR;
        mTL_angle1 = TL_angle1;
        mTL_angle2 = TL_angle2;


        // ..add a GUI slider to control throttle left via mouse
        //virtual IGUIScrollBar * addScrollBar (bool horizontal, const core::rect< s32 > &rectangle, IGUIElement *parent=0, s32 id=-1)=0
        //virtual IGUICheckBox * addCheckBox (bool checked, const core::rect< s32 > &rectangle, IGUIElement *parent=0, s32 id=-1, const wchar_t *text=0)=0
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
    
        // ..add a GUI slider to control throttle left via mouse
        scrollbar_TL1 =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(750, 20, 890, 35), 0, 105);
        scrollbar_TL1->setMax(100);
        scrollbar_TL1->setPos(25);
        text_TL1 =
            application->GetIGUIEnvironment()->addStaticText(L"tail link 1 ", rect<s32>(900, 20, 1000, 35), false);

        // ..add a GUI slider to control throttle left via mouse
        scrollbar_TL2 =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(750, 45, 890, 60), 0, 106);
        scrollbar_TL2->setMax(100);
        scrollbar_TL2->setPos(100);
        text_TL2 =
            application->GetIGUIEnvironment()->addStaticText(L"tail link 2 ", rect<s32>(900, 45, 1000, 60), false);

        // ..add a GUI checkbox
        checkbox_linkLocked = 
            application->GetIGUIEnvironment()->addCheckBox(false, rect<s32>(900, 70, 1000, 85), 0, 2110);
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

                    //後部ユニットの速度
                    mfun = std::static_pointer_cast<ChFunction_Const>(mbackunit->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst((newthrottleR * 6 + newthrottleL * 6)/2);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mbackunit->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst((newthrottleR * 6 + newthrottleL * 6)/2);

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
                if (id == 105) {  // id of 'throttleL' slider..
                    s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    *mTL_angle1 = CH_C_PI * (pos - 50) / 100;

                    return true;
                }
                if (id == 106) {  // id of 'throttleL' slider..
                    s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    *mTL_angle2 = CH_C_PI * (pos - 50) / 100;

                    return true;
                }
                break;

            case gui::EGET_CHECKBOX_CHANGED:
                if (id == 2110) {
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
    MySimpleBackunit* mbackunit;
    double* mangleL;
    double* mangleR;
    double* mTL_angle1;
    double* mTL_angle2;

    IGUIStaticText* text_throttleL;
    IGUIScrollBar* scrollbar_throttleL;
    IGUIStaticText* text_throttleR;
    IGUIScrollBar* scrollbar_throttleR;
    IGUIStaticText* text_flipperL;
    IGUIScrollBar* scrollbar_flipperL;
    IGUIStaticText* text_flipperR;
    IGUIScrollBar* scrollbar_flipperR;
    IGUIStaticText* text_TL1;
    IGUIScrollBar* scrollbar_TL1;
    IGUIStaticText* text_TL2;
    IGUIScrollBar* scrollbar_TL2;
    IGUICheckBox* checkbox_linkLocked;
};


int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);

    ChSystemNSC my_system;
    my_system.SetCollisionSystemType(collision_type);

    ChIrrApp application(&my_system, L"Modeling a simplified   tank", core::dimension2d<u32>(1200, 800));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-10, 10, 1.4), core::vector3df(-5, 5, 1.4));

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(1.0);

    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(60, 2, 60, 1000, true, true, ground_mat);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->SetBodyFixed(true);
    my_ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/blue.png")));
    my_system.AddBody(my_ground);

    my_system.Set_G_acc({ 0, -9.81, 0 });


    //------------------------------------------
    // create robot model
    //------------------------------------------
    //
    // ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)

    double model_height = 5;
    //bool fixflag = true;
    bool fixflag = false;


    MySimpleTank* mytank = new MySimpleTank(my_system, application.GetSceneManager(), application.GetVideoDriver(), -5, model_height,fixflag);
    ChVector<> center_pos = mytank->wheelLB->GetPos() - mytank->wheelLF->GetPos();

    printf("tred is %lf\r\n", mytank->wheelLF->GetPos().z() /*- mytank->wheelRF->GetPos().z()*/);
    
    MySimpleFlipper* myflipper = new MySimpleFlipper(
        my_system,
        application.GetSceneManager(),
        application.GetVideoDriver(),
        mytank->wheelLF->GetPos().x(),
         model_height, 
        0
    );

    MySimpleBackunit* mybackunit = new MySimpleBackunit(my_system, application.GetSceneManager(), application.GetVideoDriver(), -5-1.47-1-1.5, model_height, false);


    // Create the motor
    auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor1->Initialize(myflipper->trussL,                // body A (slave)
        mytank->truss,               // body B (master)
        ChFrame<>(mytank->wheelLF->GetPos())  // motor frame, in abs. coords
    );
    my_system.Add(rotmotor1);

    auto motor_funL = chrono_types::make_shared<ChFunction_Setpoint>();
    rotmotor1->SetAngleFunction(motor_funL);


    // Create the motor
    auto rotmotor2= chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor2->Initialize(myflipper->trussR,                // body A (slave)
        mytank->truss,               // body B (master)
        ChFrame<>(mytank->wheelRF->GetPos())  // motor frame, in abs. coords
    );
    my_system.Add(rotmotor2);

    auto motor_funR = chrono_types::make_shared<ChFunction_Setpoint>();
    rotmotor2->SetAngleFunction(motor_funR);



    //----------------------------
    //cleate tail mesh
    //---------------------------

    auto tailmesh = chrono_types::make_shared<ChBodyEasyMesh>(               //
        GetChronoDataFile("models/alacran_x10/back_unit_link.obj").c_str(),  // data file
        10000,                                                          // density
        true,                                                         // do not compute mass and inertia
        true,                                                          // visualization?
        false,                                                         // collision?
        nullptr,               // no need for contact material
        0);                                                            // mesh sweep sphere radius
    //tailmesh->SetBodyFixed(true);
    my_system.Add(tailmesh);
    tailmesh->SetRot(Q_from_AngAxis(0, VECT_X));
    tailmesh->SetPos(mytank->truss->GetPos() + ChVector<>(-1.47-0.865, 0, 0));
    //tailmesh->SetPos(mytank->wheelLB->GetPos());
    tailmesh->SetMass(10);

    auto color_tail = chrono_types::make_shared<ChColorAsset>();
    color_tail->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    tailmesh->AddAsset(color_tail);

    std::cout << mytank->truss->GetPos() << std::endl;
    
    // Create the motor
    auto tail_link1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    tail_link1->Initialize(tailmesh,                // body A (slave)
        mytank->truss,               // body B (master)
        //ChFrame<>(mytank->wheelLB->GetPos())  // motor frame, in abs. coords
        ChFrame<>(mytank->truss->GetPos() + ChVector<>(-1.47, 0, 0))
    );
    
    my_system.Add(tail_link1);

    auto motor_funBack1 = chrono_types::make_shared<ChFunction_Setpoint>();
    tail_link1->SetAngleFunction(motor_funBack1);


    
    auto tail_link2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();  // left, front, upper, 1
    
    tail_link2->Initialize(tailmesh, 
        mybackunit->truss,
        ChFrame<>(mytank->truss->GetPos() + ChVector<>(-1.47-1.73, 0, 0))
    );
    
    my_system.AddLink(tail_link2);

    auto motor_funBack2 = chrono_types::make_shared<ChFunction_Setpoint>();
    tail_link2->SetAngleFunction(motor_funBack2);


    //auto tail_link2 = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1

    //tail_link2->Initialize(tailmesh, mybackunit->truss,
    //    ChCoordsys<>(tailmesh->GetPos(), QUNIT));
    //my_system.AddLink(tail_link2);


    //---------------------
    // cleate field ofject
    //--------------------

    //StepType step_type = FLAT ;
    //RandomStep* myrandomstep = new RandomStep(my_system, step_type);


    //
    //---create flipper arm-------
    //
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    double angleL = 0;
    double angleR = 0;
    double TL_angle1 = CH_C_PI * (25 - 50) / 100;
    double TL_angle2 = CH_C_PI * (100 - 50) / 100;
    bool linklocked = false;

    // Create a ChFunction to be used for the ChLinkMotorRotationAngle
    auto msineangle = chrono_types::make_shared<ChFunction_Const>(0);       // phase [rad]




    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    MyEventReceiver receiver(&application,mytank, myflipper, mybackunit, &angleL, &angleR, &TL_angle1, &TL_angle2);
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

    while (application.GetDevice()->run()) {
        double t = my_system.GetChTime();
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        //application.GetActiveCamera()->setTarget(core::vector3dfCH(mytank->truss->GetPos()));


        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        tools::drawGrid(application.GetVideoDriver(), 2, 2, 30, 30,
            ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
            video::SColor(255, 60, 60, 60), true);

        //std::cout << angle << std::endl;
        //msineangle->Set_yconst(angle);
        motor_funL->SetSetpoint(angleL, 0.5);
        motor_funR->SetSetpoint(angleR, 0.5);
        motor_funBack1->SetSetpoint(TL_angle1, 2);
        motor_funBack2->SetSetpoint(TL_angle2, 2);

        application.DoStep();

        application.EndScene();
    }
    if (mytank)
        delete mytank;

    return 0;
}