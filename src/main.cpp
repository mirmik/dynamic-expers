#include <crow/functional.h>
#include <crow/gates/udpgate.h>
#include <igris/time/systime.h>
#include <igris/util/string.h>
#include <rabbit/compat/reactphysics3d.h>
#include <rabbit/mesh/mesh.h>
#include <rabbit/mesh/primitives.h>
//#include <rabbitreactphysics3d/RigidBodyDrawer.h>
//#include <rabbitreactphysics3d/RigidBodyPair.h>
#include <rabbitreactphysics3d/util.h>
#include <ralgo/filter/servo_second_tier.h>
#include <ralgo/util/math.h>
#include <reactphysics3d/reactphysics3d.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace reactphysics3d;

PhysicsCommon physicsCommon;

class Manipulator
{
public:
    PhysicsWorld *world;
    RigidBody *base_body;
    RigidBody *base_body_proxy;
    RigidBody *arm1;
    RigidBody *arm2;
    RigidBody *arm3;
    RigidBody *arm4;

    HingeJoint *joint0;
    HingeJoint *joint1;
    HingeJoint *joint2;
    HingeJoint *joint3;

    std::vector<RigidBody *> bodies = {};
    std::vector<RigidBody *> arms = {};
    std::vector<HingeJoint *> joints = {};

public:
    Manipulator(PhysicsWorld *world,
                const ralgo::pose3<float> &initial_pose = ralgo::pose3<float>())
        : world(world)
    {
        base_body = world->createRigidBody(
            rabbit::react_cast(initial_pose * ralgo::mov3<float>({0, 0, 0})));
        base_body_proxy = world->createRigidBody(
            rabbit::react_cast(initial_pose * ralgo::mov3<float>({0, 0, 0})));
        arm1 = world->createRigidBody(
            rabbit::react_cast(initial_pose * ralgo::mov3<float>({0, 0, 0})));
        arm2 = world->createRigidBody(
            rabbit::react_cast(initial_pose * ralgo::mov3<float>({0, 0, 1})));
        arm3 = world->createRigidBody(
            rabbit::react_cast(initial_pose * ralgo::mov3<float>({0, 0, 2})));

        base_body->setType(BodyType::STATIC);

        create_fixed_joint(
            world, get_pose(base_body).lin, base_body_proxy, base_body);
        joint0 = create_hinge_joint(
            world, get_pose(arm1).lin, {0, 1, 0}, arm1, base_body_proxy);
        joint1 = create_hinge_joint(
            world, get_pose(arm2).lin, {0, 1, 0}, arm2, arm1);
        joint2 = create_hinge_joint(
            world, get_pose(arm3).lin, {0, 1, 0}, arm3, arm2);
        // joint3 = create_hinge_joint(world, arm3.get_pose().lin, {0,1,0},
        // arm4.body, arm3.body);

        bodies = {base_body, base_body_proxy, arm1, arm2, arm3}; //, &arm4};
        arms = {arm1, arm2, arm3};
        joints = {joint0, joint1, joint2}; //, joint3};

        create_colliders();

        for (auto *arm : arms)
        {
            auto target = linalg::vec<float, 3>{0, 0, 2};

            auto inertia1 =
                rabbit::react_world_inertia_for_center(*arm1, target);

            auto inertia2 =
                rabbit::react_world_inertia_for_center(*arm2, target);

            auto inertia3 =
                rabbit::react_world_inertia_for_center(*arm3, target);
        }
    }

    linalg::mat<float, 3, 3> arm1_inertia()
    {
        auto target = get_pose(arm1).lin;
        auto inertia_arm1 =
            rabbit::react_world_inertia_for_center(*arm1, target);
        auto inertia_arm2 =
            rabbit::react_world_inertia_for_center(*arm2, target);
        auto inertia_arm3 =
            rabbit::react_world_inertia_for_center(*arm3, target);
        return inertia_arm1 + inertia_arm2 + inertia_arm3;
    }

    linalg::mat<float, 3, 3> arm2_inertia()
    {
        auto target = get_pose(arm2).lin;
        auto inertia_arm2 =
            rabbit::react_world_inertia_for_center(*arm2, target);
        auto inertia_arm3 =
            rabbit::react_world_inertia_for_center(*arm3, target);
        return inertia_arm2 + inertia_arm3;
    }

    linalg::mat<float, 3, 3> arm3_inertia()
    {
        auto target = get_pose(arm3).lin;
        auto inertia_arm3 =
            rabbit::react_world_inertia_for_center(*arm3, target);
        return inertia_arm3;
    }

    void add_collider_for_arm(RigidBody *arm)
    {
        auto *shape = physicsCommon.createCapsuleShape(0.25, 0.4);
        auto transform =
            rabbit::react_cast(ralgo::mov3<float>({0, 0, 0.5}) *
                               ralgo::rot3<float>({1, 0, 0}, M_PI / 2));
        arm->addCollider(shape, transform);
    }

    void create_colliders()
    {
        for (auto *body : arms)
        {
            add_collider_for_arm(body);
            body->updateMassPropertiesFromColliders();
        }
    }
};

int main()
{
    crow::create_udpgate(12);
    PhysicsWorld *world = physicsCommon.createPhysicsWorld();
    world->setGravity(Vector3(0.0, 0.0, -1.0));

    Manipulator manipulator(world);

    const int N = 3;
    ralgo::servo_second_tier servo[N];

    // set_joint_torque(manipulator.joint0, 5);

    // double T_v = 0.1, ksi_v=0.75;
    double T_p = 0.1, ksi_p = 2;

    for (int i = 0; i < N; ++i)
    {
        //    servo[i].setup_velocity_parameters(T_v, ksi_v, 0.33);
        servo[i].setup_position_parameters(T_p, ksi_p, 1);
    }

    double target[N];
    double lastang[N] = {};
    crow::start_spin();
    while (true)
    {

        double timeStep = 1.0f / 1000.0f;

        auto loct = igris::millis() % 32000;
        if (loct < 4000)
        {
            target[0] = 0;
            target[1] = 0;
            target[2] = 0;
        }
        else if (loct < 8000)
        {
            target[0] = 0;
            target[1] = 0;
            target[2] = 1;
        }
        else if (loct < 12000)
        {
            target[0] = 0;
            target[1] = 1;
            target[2] = 0;
        }
        else if (loct < 16000)
        {
            target[0] = 0;
            target[1] = 1;
            target[2] = 1;
        }
        else if (loct < 20000)
        {
            target[0] = 1;
            target[1] = 0;
            target[2] = 0;
        }
        else if (loct < 24000)
        {
            target[0] = 1;
            target[1] = 0;
            target[2] = 1;
        }
        else if (loct < 28000)
        {
            target[0] = 1;
            target[1] = 1;
            target[2] = 0;
        }
        else
        {
            target[0] = 1;
            target[1] = 1;
            target[2] = 1;
        }

        // target[0] = 1; target[1] = -1;  target[2] = 1;

        auto inertia1 = manipulator.arm1_inertia();
        auto inertia2 = manipulator.arm2_inertia();
        auto inertia3 = manipulator.arm3_inertia();
        servo[0].setup_velocity_filter(inertia1[1][1] * 30, 0);
        servo[1].setup_velocity_filter(inertia2[1][1] * 30, 0);
        servo[2].setup_velocity_filter(inertia3[1][1] * 30, 0);
        nos::println(inertia1[1][1]);
        nos::println(inertia2[1][1]);
        nos::println(inertia3[1][1]);

        for (int i = 0; i < N; ++i)
        {
            auto curang = -manipulator.joints[i]->getAngle();
            auto angvel = ralgo::angdiff<float>(curang, lastang[i]) / timeStep;
            auto torque = servo[i].mark_position_control(
                target[i], curang, angvel, 1, timeStep);
            set_joint_torque(manipulator.joints[i], torque);
            lastang[i] = curang;
        }

        crow::publish("manip/arm1/pose", get_pose_as_json(manipulator.arm1));

        world->update(timeStep);
    }
}