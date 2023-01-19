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
    bool handle_check = false;
    IGUICheckBox* checkbox_linkLocked;


    MyEventReceiver(ChIrrAppInterface* myapp, 
        MySimpleTank* atank, 
        MySimpleFlipper* aflipper, 
        MySimpleBackunit* abackunit, 
        double* angleL, 
        double* angleR,
        double* TL_angle1,
        double* TL_angle2
        ) {
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
                    handle_check;
                    auto mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst(0);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mflipper->link_motorLB->GetSpeedFunction());
                    mfun->Set_yconst(0);

            
                    mfun = std::static_pointer_cast<ChFunction_Const>(mtank->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst(0);
                    mfun = std::static_pointer_cast<ChFunction_Const>(mflipper->link_motorRB->GetSpeedFunction());
                    mfun->Set_yconst(0);
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
    
};


int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);

    ChSystemNSC my_system;
    my_system.SetCollisionSystemType(collision_type);

    ChIrrApp application(&my_system, L"Modeling a simplified   tank", core::dimension2d<u32>(1600, 800));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-10, 10, 1.4), core::vector3df(-5, 5, 1.4));

    // ..the world
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(1.0);
    //ground_mat->SetRollingFriction(0.003);
    //ground_mat->SetSpinningFriction(0.003);

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

    //double model_height = 0;
    double model_height =2;
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



    auto shoe = chrono_types::make_shared<ChBodyEasyMesh>(               //
        GetChronoDataFile("models/alacran_x10/shoe_flipper_1_collision.obj").c_str(),  // data file
        10000,                                                          // density
        true,                                                         // do not compute mass and inertia
        true,                                                          // visualization?
        false,                                                         // collision?
        nullptr,               // no need for contact material
        0);
    std::cout <<"shoe inertia" << shoe->GetInertiaXX() << std::endl;

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
    std::cout << "tail inertia" << tailmesh->GetInertiaXX() << std::endl;
    tailmesh->SetInertiaXX(ChVector<>(3.64473 , 68.5446 , 70.9417));
    //tailmesh->SetPos(mytank->wheelLB->GetPos());
    tailmesh->SetMass(10);

    auto color_tail = chrono_types::make_shared<ChColorAsset>();
    color_tail->SetColor(ChColor(0.004f, 0.004f, 0.004f));
    color_tail->SetFading(0.5f);
    tailmesh->AddAsset(color_tail);

    std::cout << mytank->truss->GetPos() << std::endl;
    
    // Create the motor
    auto tail_link1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    tail_link1->Initialize(tailmesh,                // body A (slave)
        mytank->truss,               // body B (master)
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

    
    //--------------------
    //------create imu---
    //-------------------

    auto imumesh = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.4, 0.5, 1000, true, false, chrono_types::make_shared<ChMaterialSurfaceNSC>());
    imumesh->SetPos(mytank->truss->GetPos() + ChVector<>(0, 0.45, 0));
    imumesh->SetMass(1);
    std::cout << "mass is" << imumesh->GetMass() << std::endl;
    imumesh->SetCollide(false);
    my_system.Add(imumesh);

    auto link_imu = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
    link_imu->Initialize(imumesh, mytank->truss,
        ChCoordsys<>(mytank->truss->GetPos(), QUNIT));
    link_imu->Lock(true);
    my_system.AddLink(link_imu);

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

    std::cout << 90 * CH_C_DEG_TO_RAD << std::endl;
    //auto imu_offset_pose = chrono::ChFrame<double>({ 0, 0, 0 }, Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
    auto imu_offset_pose = chrono::ChFrame<double>({ 0, 0, 0 }, Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, { 1, 1, 1 }));
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
    manager->AddSensor(gyro);                                        // Add the IMU sensor to the sensor manager

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
    auto name = "hoge";
    //auto size = 48;

    typedef struct
    {
        double t;
        double acc_data[3];
        double gyro_data[3];
        double flipper_angle[2];
    } SHARED_MEMORY_DATA;

    SHARED_MEMORY_DATA* gMappingObject = NULL;

    HANDLE hSharedMemory = CreateFileMapping(NULL, NULL, PAGE_READWRITE, NULL, sizeof(SHARED_MEMORY_DATA), name);
    auto pMemory = (SHARED_MEMORY_DATA*)MapViewOfFile(hSharedMemory, FILE_MAP_ALL_ACCESS, NULL, NULL, sizeof(SHARED_MEMORY_DATA));


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
        if(receiver.checkbox_linkLocked->isChecked()){
            motor_funL->SetSetpoint(angleL, 2);
            motor_funR->SetSetpoint(angleR, 2);
        }
        else {
            motor_funL->SetSetpoint(pMemory->flipper_angle[0], 2);
            motor_funR->SetSetpoint(pMemory->flipper_angle[1], 2);
        }
        motor_funBack1->SetSetpoint(TL_angle1, 2);
        motor_funBack2->SetSetpoint(TL_angle2, 2);

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
                else if (i == 1)ss[i] << L"Y acc:";
                else ss[i] << L"Z acc:";

                ss[i] << acc_data[i];
                acc_data_text[i]->setText(ss[i].str().c_str());

            }

            //imu_csv << std::fixed << std::setprecision(6);
            pMemory->t = t;
            pMemory->acc_data[0] = -acc_data[0]/10.0;
            pMemory->acc_data[1] = -acc_data[2]/10.0;
            pMemory->acc_data[2] = -acc_data[1]/10.0;
            pMemory->gyro_data[0] = gyro_data.Roll;
            pMemory->gyro_data[1] = gyro_data.Yaw; 
            pMemory->gyro_data[2] = gyro_data.Pitch;
            imu_last_launch = bufferGyro->LaunchedCount;
            //printf("%0.4f %0.4f %0.4f \n", acc_data.X, acc_data.Y, acc_data.Z);
        }
        manager->Update();

        application.DoStep();

        application.EndScene();
    }
    UnmapViewOfFile(pMemory);
    CloseHandle(hSharedMemory);
    if (mytank)
        delete mytank;

    return 0;
}