#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

void CreateStatorRotor(std::shared_ptr<ChBody>& mstator,
    std::shared_ptr<ChBody>& mrotor,
    std::shared_ptr<ChMaterialSurface> material,
    ChSystem& msystem,
    const ChVector<>& mpos) {
    //mstator = chrono_types::make_shared<ChBodyEasyCylinder>(0.5, 0.1, 1000, material, collision_type);
    mstator = chrono_types::make_shared<ChBodyEasyBox>(1, 0.1, 1, 3000, material, collision_type);
    mstator->SetPos(mpos);
    mstator->SetCollide(false);
    mstator->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    mstator->SetBodyFixed(false);
    msystem.Add(mstator);

    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("bluewhite.png"));
    mstator->AddAsset(texture);

    mrotor = chrono_types::make_shared<ChBodyEasyBox>(2, 0.1, 0.1, 10000, material, collision_type);
    mrotor->SetPos(mpos + ChVector<>(1, 0, -0.15));
    msystem.Add(mrotor);

    auto mcolor = chrono_types::make_shared<ChColorAsset>(0.6f, 0.6f, 0.0f);
    mrotor->AddAsset(mcolor);
}

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
public:
    MyEventReceiver(ChIrrAppInterface* myapp, double* angle) {
        // store pointer application
        application = myapp;
        mangle = angle;
        // ..add a GUI slider to control throttle left via mouse
        scrollbar_throttleL =
            application->GetIGUIEnvironment()->addScrollBar(true, rect<s32>(510, 20, 650, 35), 0, 101);
        scrollbar_throttleL->setMax(100);
        scrollbar_throttleL->setPos(50);
        text_throttleL =
            application->GetIGUIEnvironment()->addStaticText(L"flipper angle ", rect<s32>(650, 20, 750, 35), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();

            switch (event.GUIEvent.EventType) {
            case EGET_SCROLL_BAR_CHANGED:
                if (id == 101) {  // id of 'throttleL' slider..
                    s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                    *mangle = CH_C_PI*pos/50;
                    
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
    //ChFunction_Const* mangle;
    double* mangle;

    IGUIStaticText* text_throttleL;
    IGUIScrollBar* scrollbar_throttleL;
    IGUIStaticText* text_throttleR;
    IGUIScrollBar* scrollbar_throttleR;
};

int main(int argc, char* argv[]) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    GetLog() << "CH_C_PI: " << CH_C_PI << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;
    mphysicalSystem.SetCollisionSystemType(collision_type);

    // Contact material shared among all objects
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    double angle = 0.0;
    double torque = 0.0;

    // Create a floor that is fixed (that is used also to represent the absolute reference)
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, material, collision_type);
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);
    mphysicalSystem.Add(floorBody);

    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/blue.png"));
    floorBody->AddAsset(mtexture);

    auto box = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 2, 30000, material, collision_type);
    box->SetPos(ChVector < >(0,0,0));
    box->SetMass(100);
    box->SetBodyFixed(false);
    mphysicalSystem.Add(box);


    ChVector<> positionA2(0, 0, -1);
    std::shared_ptr<ChBody> stator2;
    std::shared_ptr<ChBody> rotor2;
    CreateStatorRotor(stator2, rotor2, material, mphysicalSystem, positionA2);

    // Create the motor
    auto rotmotor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

    // Connect the rotor and the stator and add the motor to the system:
    rotmotor2->Initialize(rotor2,                // body A (slave)
        stator2,               // body B (master)
        ChFrame<>(positionA2)  // motor frame, in abs. coords
    );
    mphysicalSystem.Add(rotmotor2);


    GetLog() << "get pos: " << rotor2->GetPos() << "\n\n";

    // Create a ChFunction to be used for the ChLinkMotorRotationAngle
    auto msineangle = chrono_types::make_shared<ChFunction_Const>(1.5);       // phase [rad]
    auto motor_fun = chrono_types::make_shared<ChFunction_Setpoint>();

    //auto msineangle = chrono_types::make_shared<ChFunction_Sine>(0,       // phase [rad]
    //    0.05,    // frequency [Hz]
    //    CH_C_PI  // amplitude [rad]
    //    );

    // Let the motor use this motion function as a motion profile:
    rotmotor2->SetAngleFunction(motor_fun);


    std::shared_ptr<ChLinkLockRevolute> link1;
    link1 = chrono_types::make_shared<ChLinkLockRevolute>();
    link1->Initialize(stator2, box, ChCoordsys<>(ChVector<>(1,1,1), QUNIT));
    link1->Lock(true);
    mphysicalSystem.AddLink(link1);

    //
    //----Torque test
    //

    //ChVector<> positionA3(-3, 2, -1);
    //std::shared_ptr<ChBody> stator3;
    //std::shared_ptr<ChBody> rotor3;
    //CreateStatorRotor(stator3, rotor3, material, mphysicalSystem, positionA3);
    //stator3->SetBodyFixed(true);
    //rotor3->SetMass(10);

    //// Create the motor
    //auto rotmotor3 = chrono_types::make_shared<ChLinkMotorRotationTorque>();

    //// Connect the rotor and the stator and add the motor to the system:
    //rotmotor3->Initialize(rotor3,                // body A (slave)
    //    stator3,               // body B (master)
    //    ChFrame<>(positionA3)  // motor frame, in abs. coords
    //);
    //mphysicalSystem.Add(rotmotor3);

    //// The torque(time) function:
    //auto mtorquetime = chrono_types::make_shared<ChFunction_Const>(1.5);

    //// Let the motor use this motion function as a motion profile:
    //rotmotor3->SetTorqueFunction(mtorquetime);

    //-------------------------------------------------------------------------------------------------

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Motors", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1, 3, -7));
    application.AddLightWithShadow(vector3df(20.0f, 35.0f, -25.0f), vector3df(0, 0, 0), 55, 20, 55, 35, 512,
        video::SColorf(0.6f, 0.8f, 1.0f));

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
    MyEventReceiver receiver(&application, &angle);
    // note how to add the custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
    // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
    application.AddShadowAll();

    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(50);

    application.SetTimestep(0.005);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {

        
        application.BeginScene(true, true, video::SColor(255, 140, 161, 192));

        application.DrawAll();

        // Example B.6 requires the setpoint to be changed in the simulation loop:
        // for example use a clamped sinusoid, just for fun:
        double t = mphysicalSystem.GetChTime();
        //torque = rotmotor3->GetMotorRotPeriodic();
        if (torque < 0)torque = 2 * CH_C_PI + torque;
        torque = angle - torque;
        
        if (torque < -CH_C_PI) torque += 2 * CH_C_PI;
        else if (torque > CH_C_PI) torque += -2 * CH_C_PI;

        if (abs(torque) > 0.2) torque = torque * 10000;

        std::cout << angle << ":"<<torque << std::endl;
        msineangle->Set_yconst(angle);
        motor_fun->SetSetpoint(angle, 0.5);
        //mtorquetime->Set_yconst(torque);


        application.DoStep();

        application.EndScene();
    }

    return 0;


}