
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
        IVideoDriver* mdriver,         ///< the Irrlicht video driver
        double mx,
        double mz,
        bool fixflag
    ) {
        throttleL = throttleR = 0;  // initially, gas throttle is 0.
        max_motor_speed = 10;

        //double my = 0.5;  // left back hub pos
        //double mx = 0;

        //double shoelength = 0.0375; //ベルトユニットの長さ
        double shoelength = 0.42; //ベルトユニットの長さ
        //double shoe2length = 0.28; //ベルトユニットの長さ
        double shoemass = 3;       //ベルトユニットの重さ
        double radiustrack = 0.52; //クローラの半径
        double wheeldiameter = 0.47 * 2; //プーリーの直径 初期0.045
        int nwrap = 6;  //クローラの曲がってるとこのユニットの数
        int ntiles = 6; //まっすぐなとこのユニットの数
        double rlwidth = 2.16;
        double passo = (ntiles + 1.0) * (shoelength);
        printf("passo is % lf\r\n", passo);
        //double passo = (ntiles + 1) * (shoelength);

        ChVector<> cyl_displA(0, 0.4, 0); //円柱の中心の座標
        ChVector<> cyl_displB(0, -0.4, 0);
        double cyl_hthickness = 0.30; //0.03だと高さ0.06の円柱になる

        //ChVector<> cyl_displA(0, 0.0325, 0);
        //ChVector<> cyl_displB(0, -0.0325, 0);
        //double cyl_hthickness = 0.02;

        // --- The tank body ---
        auto truss_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        truss_mat->SetFriction(0.0);//摩擦係数

        auto load_truss = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/truss.obj").c_str(),   // data file
            1000,                                                          // density
            true,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            true,                                                         // collision?
            truss_mat,                                                       // no need for contact material
            0.0);                                                            // mesh sweep sphere radius
        //my_system.AddBody(Lflipper);
        load_truss->SetPos(ChVector<>(mx + passo / 2, rlwidth / 2, mz + radiustrack));
        load_truss->SetRot(Q_from_AngAxis(-180 * CH_C_DEG_TO_RAD, VECT_X));
        load_truss->SetBodyFixed(fixflag);
        load_truss->SetMass(700);
        load_truss->SetInertiaXX(ChVector<>(13.8, 13.5, 10));
        //load_truss->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));


        truss = load_truss;

        auto color_truss = chrono_types::make_shared<ChColorAsset>();
        color_truss->SetColor(ChColor(0.004f, 0.004f, 0.004f));
        truss->AddAsset(color_truss);

        my_system.Add(truss);

        //auto color_truss = chrono_types::make_shared<ChColorAsset>();
        //color_truss->SetColor(ChColor(1.0f, 0.2f, 0.2f));
        //truss->AddAsset(color_truss);


        // --- Contact material for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        wheel_mat->SetFriction(1.0);//摩擦係数

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/wheel.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, 0, mz + radiustrack));
        wheelRF->SetRot(Q_from_AngAxis(0, VECT_Y));
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
        color_wheel->SetColor(ChColor(0.004f, 0.004f, 0.004f));
        wheelRF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss, ChCoordsys<>(ChVector<>(mx + passo, 0, mz + radiustrack), Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X)));
        my_system.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/wheel.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, rlwidth, mz + radiustrack));
        wheelLF->SetRot(Q_from_AngAxis(0, VECT_Y));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        //wheelLF->AddAsset(color_wheel);

        wheelLF->GetCollisionModel()->ClearModel();
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelLF->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelLF->GetCollisionModel()->BuildModel();
        wheelLF->SetCollide(true);

        auto color_wheel1 = chrono_types::make_shared<ChColorAsset>();
        color_wheel1->SetColor(ChColor(0.004f, 0.004f, 0.004f));
        wheelLF->AddAsset(color_wheel1);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, truss,
            ChCoordsys<>(ChVector<>(mx + passo, rlwidth, mz + radiustrack), Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X)));
        my_system.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/alacran_x10/wheel.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, 0, mz + radiustrack));
        wheelRB->SetRot(Q_from_AngAxis(0, VECT_Y));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        //wheelRB->AddAsset(color_wheel);

        wheelRB->GetCollisionModel()->ClearModel();
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelRB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelRB->GetCollisionModel()->BuildModel();
        wheelRB->SetCollide(true);

        wheelRB->AddAsset(color_wheel1);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, truss, ChFrame<>(ChVector<>(mx, 0, mz + radiustrack), Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X)));
        my_system.AddLink(link_motorRB);

        // --- Left Back suspension ---

        // ..the tank left-back wheel

        wheelLB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/alacran_x10/wheel.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, rlwidth, mz + radiustrack));
        wheelLB->SetRot(Q_from_AngAxis(0, VECT_Y));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        //wheelLB->AddAsset(color_wheel);

        wheelLB->GetCollisionModel()->ClearModel();
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displA);
        wheelLB->GetCollisionModel()->AddCylinder(wheel_mat, wheeldiameter / 2, wheeldiameter / 2, cyl_hthickness,
            cyl_displB);
        wheelLB->GetCollisionModel()->BuildModel();
        wheelLB->SetCollide(true);

        wheelLB->AddAsset(color_wheel1);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, truss, ChFrame<>(ChVector<>(mx, rlwidth, mz + radiustrack), Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X)));
        my_system.AddLink(link_motorLB);

        //--- TRACKS ---

        // Load a triangle mesh for collision

        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("models/alacran_x10/body_shoe_1_Zaxis_collision.obj").c_str());

        auto trimesh = chrono_types::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh.get(), irmesh_shoe_collision->getMesh(0));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

        chrono::ChVector<> position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            mx += shoelength;

            double my = 0;

            if (side == 0)
                my = 0;
            else
                my = rlwidth;

            position.Set(mx, my, mz);
            rotation = Q_from_AngAxis(0 * CH_C_DEG_TO_RAD, VECT_X);


            //------------------------------------
            //---first body shoe------------------
            //------------------------------------


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
            shoe_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/body_shoe_1_Zaxis.obj").c_str());
            shoe_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_coll_mesh);
            shoe_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/body_shoe_1_Zaxis_collision.obj").c_str());
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

            //--------------------------------------------------------------------------------
            //------------------------------------
            //---second body shoe-----------------
            //------------------------------------
            position.Set(mx + shoelength, my, mz);

            auto secondBodyShoe = chrono_types::make_shared<ChBody>();
            my_system.Add(secondBodyShoe);
            secondBodyShoe->SetMass(shoemass);
            //secondBodyShoe->SetPos(position);
            secondBodyShoe->SetPos(ChVector<>(60, 2, 60));
            secondBodyShoe->SetRot(rotation);
            secondBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            auto shoe2_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            secondBodyShoe->AddAsset(shoe2_mesh);
            shoe2_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/body_shoe_2_Zaxis.obj").c_str());
            shoe2_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1)); //位置がずれていたらここが怪しい
            shoe2_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe2_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            secondBodyShoe->AddAsset(shoe2_coll_mesh);
            shoe2_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/body_shoe_2_Zaxis_collision.obj").c_str());
            shoe2_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe2_coll_mesh->SetVisible(false);

            // Collision:
            secondBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            secondBodyShoe->GetCollisionModel()->SetEnvelope(0.010);    // distance of the outward "collision envelope"
            secondBodyShoe->GetCollisionModel()->ClearModel();
            secondBodyShoe->GetCollisionModel()->AddTriangleMesh(chrono_types::make_shared<ChMaterialSurfaceNSC>(),
                trimesh, false, false, mesh_displacement,
                ChMatrix33<>(1), 0.005);
            secondBodyShoe->GetCollisionModel()->BuildModel();  // Creates the collision model
            secondBodyShoe->SetCollide(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            secondBodyShoe->GetCollisionModel()->SetFamily(3);
            secondBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
            //------------------------------------------------------------------------------------

            //previous_rigidBodyShoe = secondBodyShoe;

            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                if (nshoe % 2 == 0) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                if (nshoe % 2 == 0) {
                    double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);
                    printf("alpha = %lf\r\n", alpha);
                    double lx = mx + shoelength + radiustrack * sin(alpha);
                    double ly = mz + radiustrack - radiustrack * cos(alpha);
                    position.Set(lx, my, ly);
                    rotation = chrono::Q_from_AngAxis(-alpha, ChVector<>(0, 1, 0));
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);
                    printf("alpha = %lf\r\n", alpha);
                    double lx = mx + shoelength + radiustrack * sin(alpha);
                    double ly = mz + radiustrack - radiustrack * cos(alpha);
                    position.Set(lx, my, ly);
                    rotation = chrono::Q_from_AngAxis(-alpha, ChVector<>(0, 1, 0));
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my , mz + 2 * radiustrack);

                if (nshoe % 2 == 1) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {

                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_C_PI + (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * sin(alpha);
                double ly = mz + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, my, ly);
                rotation = chrono::Q_from_AngAxis(-alpha, ChVector<>(0, 1, 0));
                if (nshoe % 2 == 0) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
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
    std::shared_ptr<ChBody> trussR; //右クローラー
    std::shared_ptr<ChBody> trussL; //左クローラー
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

    collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;


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

        double shoelength = 0.33;
        double shoethickness = 0.25;
        double shoemass = 3;
        double radiustrack = 0.52;
        double wheeldiameter = 0.47 * 2; //タイヤ直径
        int nwrap = 6;
        int ntiles = 6;
        double rlwidth = 2.16;
        double clwidth = 4.31;
        double passo = (ntiles + 1) * shoelength;
        printf("flipper passo %lf\r\n", passo);

        //ChVector<> cyl_displA(0, 0.075 + 0.02, 0);
        //ChVector<> cyl_displB(0, -0.075 - 0.02, 0);
        //double cyl_hthickness = 0.045;

        ChVector<> cyl_displA(0, 0.1, 0);
        ChVector<> cyl_displB(0, -0.1, 0);
        double cyl_hthickness = 0.05;

        // --- The tank body ---

        trussR = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/flipper_truss_wall.obj").c_str(),   // data file
            1000,                                                          // density
            true,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            chrono_types::make_shared<ChMaterialSurfaceNSC>(),                                                       // no need for contact material
            0.0);                                                            // mesh sweep sphere radius
        trussR->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, mz + (rlwidth / 2) - (clwidth / 2) + (shoethickness)));
        trussR->SetMass(100);
        //trussR->SetInertiaXX(ChVector<>(13.8, 13.5, 10));



        //trussR = std::make_shared<ChBodyEasyBox>(passo, wheeldiameter / 2, 0.01,  // x, y, z dimensions
        //    10000,       // density
        //    chrono_types::make_shared<ChMaterialSurfaceNSC>(),       // create visualization asset
        //    collision_type       // no collision geometry
        //    );

        //trussR->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, mz + (rlwidth / 2) - (clwidth / 2) + (shoethickness)));
        //trussR->SetMass(100);
        //trussR->SetBodyFixed(true);
        //trussR->SetCollide(false);

        auto color_truss = chrono_types::make_shared<ChColorAsset>();
        color_truss->SetColor(ChColor(0.004f, 0.004f, 0.004f));
        trussR->AddAsset(color_truss);

        my_system.Add(trussR);


        trussL = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/flipper_truss_wall.obj").c_str(),   // data file
            1000,                                                          // density
            true,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            chrono_types::make_shared<ChMaterialSurfaceNSC>(),                                                       // no need for contact material
            0.0);                                                            // mesh sweep sphere radius
        //my_system.AddBody(Lflipper);
        trussL->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, mz + (rlwidth / 2) + (clwidth / 2) - (shoethickness)));
        trussL->SetMass(100);

        //trussL = std::make_shared<ChBodyEasyBox>(passo, wheeldiameter / 2, 0.01,  // x, y, z dimensions
        //    10000,       // density
        //    chrono_types::make_shared<ChMaterialSurfaceNSC>(),       // create visualization asset
        //    collision_type       // no collision geometry
        //    );
        //trussL->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, mz + (rlwidth / 2) + (clwidth / 2) - (shoethickness)));
        //trussL->SetMass(100);
        //trussL->SetBodyFixed(false);
        //trussL->SetCollide(false);

        trussL->AddAsset(color_truss);

        my_system.Add(trussL);


        // --- Contact material for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        wheel_mat->SetFriction(1.0);

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, my + radiustrack, mz + rlwidth / 2 - clwidth / 2));
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
        color_wheel->SetColor(ChColor(0.004f, 0.004f, 0.004f));
        wheelRF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, trussR, ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, mz + rlwidth / 2 - clwidth / 2), QUNIT));
        my_system.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, my + radiustrack, mz + rlwidth / 2 + clwidth / 2));
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

        wheelLF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteLF = chrono_types::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, trussL,
            ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, mz + rlwidth / 2 + clwidth / 2), QUNIT));
        my_system.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, my + radiustrack, mz + rlwidth / 2 - clwidth / 2));
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

        wheelRB->AddAsset(color_wheel);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorRB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorRB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorRB->Initialize(wheelRB, trussR, ChFrame<>(ChVector<>(mx, my + radiustrack, mz + rlwidth / 2 - clwidth / 2), QUNIT));
        my_system.AddLink(link_motorRB);

        // --- Left Back suspension ---

        // ..the tank left-back wheel

        wheelLB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, my + radiustrack, mz + rlwidth / 2 + clwidth / 2));
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

        wheelLB->AddAsset(color_wheel);

        // .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
        link_motorLB = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        link_motorLB->SetSpeedFunction(
            chrono_types::make_shared<ChFunction_Const>());  // actually, default function type
        link_motorLB->Initialize(wheelLB, trussL, ChFrame<>(ChVector<>(mx, my + radiustrack, mz + rlwidth / 2 + clwidth / 2), QUNIT));
        my_system.AddLink(link_motorLB);

        //--- TRACKS ---

        // Load a triangle mesh for collision

        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_1_collision.obj").c_str());
        IAnimatedMesh* irmesh_shoe_collision2 = msceneManager->getMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2_collision.obj").c_str());

        auto trimesh = chrono_types::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh.get(), irmesh_shoe_collision->getMesh(0));
        auto trimesh2 = chrono_types::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh2.get(), irmesh_shoe_collision2->getMesh(0));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

        chrono::ChVector<> position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            //mx = 0;
            mx += shoelength;

            if (side == 0)
                mz = mz + rlwidth / 2 - clwidth / 2;
            else
                mz = mz + clwidth;

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
            shoe_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_1.obj").c_str());
            shoe_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_coll_mesh);
            shoe_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_1_collision.obj").c_str());
            shoe_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_coll_mesh->SetVisible(false);

            // Collision:
            //firstBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            //firstBodyShoe->GetCollisionModel()->SetEnvelope(0.0010);    // distance of the outward "collision envelope"

            //firstBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            //firstBodyShoe->GetCollisionModel()->SetEnvelope(0.005);    // distance of the outward "collision envelope"

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

            //--------------------------------------------------------------------------------
            //------------------------------------
            //---second body shoe-----------------
            //------------------------------------
            position.Set(mx + shoelength, my, mz);

            auto secondBodyShoe = chrono_types::make_shared<ChBody>();
            my_system.Add(secondBodyShoe);
            secondBodyShoe->SetMass(shoemass);
            //secondBodyShoe->SetPos(position);
            secondBodyShoe->SetPos(ChVector<>(60, 2, 60));
            secondBodyShoe->SetRot(rotation);
            secondBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            auto shoe2_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            secondBodyShoe->AddAsset(shoe2_mesh);
            shoe2_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2.obj").c_str());
            shoe2_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1)); //位置がずれていたらここが怪しい
            shoe2_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe2_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            secondBodyShoe->AddAsset(shoe2_coll_mesh);
            shoe2_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2_collision.obj").c_str());
            shoe2_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe2_coll_mesh->SetVisible(false);

            // Collision:
            //secondBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            //secondBodyShoe->GetCollisionModel()->SetEnvelope(0.0010);    // distance of the outward "collision envelope"

            //secondBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            //secondBodyShoe->GetCollisionModel()->SetEnvelope(0.005);    // distance of the outward "collision envelope"

            secondBodyShoe->GetCollisionModel()->ClearModel();
            secondBodyShoe->GetCollisionModel()->AddTriangleMesh(chrono_types::make_shared<ChMaterialSurfaceNSC>(),
                trimesh2, false, false, mesh_displacement,
                ChMatrix33<>(1), 0.005);
            secondBodyShoe->GetCollisionModel()->BuildModel();  // Creates the collision model
            secondBodyShoe->SetCollide(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            secondBodyShoe->GetCollisionModel()->SetFamily(3);
            secondBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
            //------------------------------------------------------------------------------------


            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                if (nshoe % 2 == 0) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                if (nshoe % 2 == 0) {
                    double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);
                    printf("alpha = %lf\r\n", alpha);
                    double lx = mx + shoelength + radiustrack * sin(alpha);
                    double ly = my + radiustrack - radiustrack * cos(alpha);
                    position.Set(lx, ly, mz);
                    rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);
                    printf("alpha = %lf\r\n", alpha);
                    double lx = mx + shoelength + radiustrack * sin(alpha);
                    double ly = my + radiustrack - radiustrack * cos(alpha);
                    position.Set(lx, ly, mz);
                    rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                if (nshoe % 2 == 1) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {

                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_C_PI + (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                if (nshoe % 2 == 0) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
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

class MySimpleBackunit {
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
    MySimpleBackunit(ChSystemNSC& my_system,        ///< the Chrono physical system
        ISceneManager* msceneManager,  ///< the Irrlicht scene manager for 3d shapes
        IVideoDriver* mdriver,         ///< the Irrlicht video driver
        double mx,
        double my,
        bool fixflag
    ) {
        throttleL = throttleR = 0;  // initially, gas throttle is 0.
        max_motor_speed = 10;

        //double my = 0.5;  // left back hub pos
        //double mx = 0;

        //double shoelength = 0.0375; //ベルトユニットの長さ
        double shoelength = 0.33; //ベルトユニットの長さ
        //double shoe2length = 0.28; //ベルトユニットの長さ
        double shoemass = 1;       //ベルトユニットの重さ
        double radiustrack = 0.52; //クローラの半径
        double wheeldiameter = 0.45 * 2; //プーリーの直径 初期0.045
        int nwrap = 6;  //クローラの曲がってるとこのユニットの数
        int ntiles = 5; //まっすぐなとこのユニットの数
        double rlwidth = 2.16;
        double clwidth = 0.75;
        double passo = (ntiles + 1.0) * (shoelength);
        printf("back unit passo is % lf\r\n", passo);
        //double passo = (ntiles + 1) * (shoelength);

        ChVector<> cyl_displA(0, 0.1, 0);
        ChVector<> cyl_displB(0, -0.1, 0);
        double cyl_hthickness = 0.05;

        //ChVector<> cyl_displA(0, 0.0325, 0);
        //ChVector<> cyl_displB(0, -0.0325, 0);
        //double cyl_hthickness = 0.02;

        // --- The tank body ---
        auto truss_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        truss_mat->SetFriction(0.0);//摩擦係数

        auto load_truss = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/back_unit_truss.obj").c_str(),   // data file
            1000,                                                          // density
            true,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            true,                                                         // collision?
            truss_mat,                                                       // no need for contact material
            0.0);                                                            // mesh sweep sphere radius
        //my_system.AddBody(Lflipper);
        load_truss->SetPos(ChVector<>(mx + passo / 2, my + radiustrack, rlwidth / 2));
        load_truss->SetBodyFixed(fixflag);
        load_truss->SetMass(10);


        truss = load_truss;

        auto color_truss = chrono_types::make_shared<ChColorAsset>();
        color_truss->SetColor(ChColor(0.1f, 0.1f, 0.1f));
        truss->AddAsset(color_truss);

        my_system.Add(truss);

        //auto color_truss = chrono_types::make_shared<ChColorAsset>();
        //color_truss->SetColor(ChColor(1.0f, 0.2f, 0.2f));
        //truss->AddAsset(color_truss);


        // --- Contact material for wheels ---

        auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        wheel_mat->SetFriction(1.0);//摩擦係数

        // --- Right Front suspension ---

        // ..the tank right-front wheel
        wheelRF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelRF);
        wheelRF->SetPos(ChVector<>(mx + passo, my + radiustrack, rlwidth / 2 - clwidth / 2));
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
        color_wheel->SetColor(ChColor(0.0f, 0.0f, 0.7f));
        wheelRF->AddAsset(color_wheel);

        // .. create the revolute joint between the wheel and the truss
        link_revoluteRF = chrono_types::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, truss, ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, rlwidth / 2 - clwidth / 2), QUNIT));
        my_system.AddLink(link_revoluteRF);

        // --- Left Front suspension ---

        // ..the tank left-front wheel

        wheelLF = chrono_types::make_shared<ChBodyEasyMesh>(               //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),  // data file
            1000,                                                          // density
            false,                                                         // do not compute mass and inertia
            true,                                                          // visualization?
            false,                                                         // collision?
            nullptr,                                                       // no need for contact material
            0);                                                            // mesh sweep sphere radius

        my_system.Add(wheelLF);
        wheelLF->SetPos(ChVector<>(mx + passo, my + radiustrack, rlwidth / 2 + clwidth / 2));
        wheelLF->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLF->SetMass(9.0);
        wheelLF->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        //wheelLF->AddAsset(color_wheel);

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
            ChCoordsys<>(ChVector<>(mx + passo, my + radiustrack, rlwidth / 2 + clwidth / 2), QUNIT));
        my_system.AddLink(link_revoluteLF);

        // --- Right Back suspension ---

        // ..the tank right-back wheel

        wheelRB = chrono_types::make_shared<ChBodyEasyMesh>(  //
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelRB);
        wheelRB->SetPos(ChVector<>(mx, my + radiustrack, rlwidth / 2 - clwidth / 2));
        wheelRB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelRB->SetMass(9.0);
        wheelRB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        //wheelRB->AddAsset(color_wheel);

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
            GetChronoDataFile("models/alacran_x10/wheel_flipper.obj").c_str(),      // data file
            1000,                                             // density
            false,                                            // do not compute mass and inertia
            true,                                             // visualization?
            false,                                            // collision?
            nullptr,                                          // no need for contact material
            0);                                               // mesh sweep sphere radius

        my_system.Add(wheelLB);
        wheelLB->SetPos(ChVector<>(mx, my + radiustrack, rlwidth / 2 + clwidth / 2));
        wheelLB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
        wheelLB->SetMass(9.0);
        wheelLB->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
        //wheelLB->AddAsset(color_wheel);

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
        link_motorLB->Initialize(wheelLB, 
            truss, ChFrame<>(ChVector<>(mx, my + radiustrack, rlwidth / 2 + clwidth / 2), QUNIT));
        my_system.AddLink(link_motorLB);

        //--- TRACKS ---

        // Load a triangle mesh for collision

        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2_collision.obj").c_str());

        auto trimesh = chrono_types::make_shared<ChTriangleMeshSoup>();
        fillChTrimeshFromIrlichtMesh(trimesh.get(), irmesh_shoe_collision->getMesh(0));

        ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);    // as mesh origin is not in body center of mass
        ChVector<> joint_displacement(-shoelength * 0.5, 0, 0);  // pos. of shoe-shoe constraint, relative to COG.

        chrono::ChVector<> position;
        chrono::ChQuaternion<> rotation;

        for (int side = 0; side < 2; side++) {
            //mx = 0;
            mx += shoelength;

            double mz = 0;

            if (side == 0)
                mz = mz + rlwidth / 2 - clwidth / 2;
            else
                mz = mz + rlwidth / 2 + clwidth / 2;

            position.Set(mx, my, mz);
            rotation = QUNIT;


            //------------------------------------
            //---first body shoe------------------
            //------------------------------------


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
            shoe_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2.obj").c_str());
            shoe_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            firstBodyShoe->AddAsset(shoe_coll_mesh);
            shoe_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2_collision.obj").c_str());
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

            //--------------------------------------------------------------------------------
            //------------------------------------
            //---second body shoe-----------------
            //------------------------------------
            position.Set(mx + shoelength, my, mz);

            auto secondBodyShoe = chrono_types::make_shared<ChBody>();
            my_system.Add(secondBodyShoe);
            secondBodyShoe->SetMass(shoemass);
            //secondBodyShoe->SetPos(position);
            secondBodyShoe->SetPos(ChVector<>(60, 2, 60));
            secondBodyShoe->SetRot(rotation);
            secondBodyShoe->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

            // Visualization:
            auto shoe2_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            secondBodyShoe->AddAsset(shoe2_mesh);
            shoe2_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2.obj").c_str());
            shoe2_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1)); //位置がずれていたらここが怪しい
            shoe2_mesh->SetVisible(true);

            // Visualize collision mesh
            auto shoe2_coll_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
            secondBodyShoe->AddAsset(shoe2_coll_mesh);
            shoe2_coll_mesh->GetMesh()->LoadWavefrontMesh(GetChronoDataFile("models/alacran_x10/shoe_flipper_2_collision.obj").c_str());
            shoe2_coll_mesh->GetMesh()->Transform(-mesh_displacement, ChMatrix33<>(1));
            shoe2_coll_mesh->SetVisible(false);

            // Collision:
            secondBodyShoe->GetCollisionModel()->SetSafeMargin(0.004);  // inward safe margin
            secondBodyShoe->GetCollisionModel()->SetEnvelope(0.010);    // distance of the outward "collision envelope"
            secondBodyShoe->GetCollisionModel()->ClearModel();
            secondBodyShoe->GetCollisionModel()->AddTriangleMesh(chrono_types::make_shared<ChMaterialSurfaceNSC>(),
                trimesh, false, false, mesh_displacement,
                ChMatrix33<>(1), 0.005);
            secondBodyShoe->GetCollisionModel()->BuildModel();  // Creates the collision model
            secondBodyShoe->SetCollide(true);

            // Avoid creation of contacts with neighbouring shoes, using
            // a collision family (=3) that does not collide with itself
            secondBodyShoe->GetCollisionModel()->SetFamily(3);
            secondBodyShoe->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
            //------------------------------------------------------------------------------------

            //previous_rigidBodyShoe = secondBodyShoe;

            for (int nshoe = 1; nshoe < ntiles; nshoe++) {
                mx += shoelength;
                position.Set(mx, my, mz);

                if (nshoe % 2 == 0) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                if (nshoe % 2 == 0) {
                    double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);
                    printf("alpha = %lf\r\n", alpha);
                    double lx = mx + shoelength + radiustrack * sin(alpha);
                    double ly = my + radiustrack - radiustrack * cos(alpha);
                    position.Set(lx, ly, mz);
                    rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    double alpha = (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);
                    printf("alpha = %lf\r\n", alpha);
                    double lx = mx + shoelength + radiustrack * sin(alpha);
                    double ly = my + radiustrack - radiustrack * cos(alpha);
                    position.Set(lx, ly, mz);
                    rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
            }
            for (int nshoe = (ntiles - 1); nshoe >= 0; nshoe--) {
                position.Set(mx, my + 2 * radiustrack, mz);

                if (nshoe % 2 == 1) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {

                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }

                mx -= shoelength;
            }
            for (int nshoe = 0; nshoe < nwrap; nshoe++) {
                double alpha = CH_C_PI + (CH_C_PI / ((double)(nwrap - 1.0))) * ((double)nshoe);

                double lx = mx + 0 + radiustrack * sin(alpha);
                double ly = my + radiustrack - radiustrack * cos(alpha);
                position.Set(lx, ly, mz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                if (nshoe % 2 == 0) {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, firstBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
                else {
                    auto rigidBodyShoe =
                        MakeShoe(previous_rigidBodyShoe, secondBodyShoe, position, rotation, my_system, joint_displacement);

                    previous_rigidBodyShoe = rigidBodyShoe;
                }
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
    ~MySimpleBackunit() {
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