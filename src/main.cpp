#include <reactphysics3d/reactphysics3d.h>
#include <rabbit/opengl/qtscene.h>
#include <rabbit/opengl/window.h>
#include <rabbit/compat/reactphysics3d.h>
#include <rabbit/mesh/mesh.h> 
#include <rabbit/mesh/primitives.h> 
#include <ralgo/filter/servo_second_tier.h>
#include <ralgo/util/math.h>
#include <igris/time/systime.h>

#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using namespace reactphysics3d;

PhysicsCommon physicsCommon;

class RigidBodyDrawer : public rabbit::drawable_object 
{ 
    RigidBody* m_body;
    rabbit::mesh<float> m_mesh;

public:
    RigidBodyDrawer(RigidBody* body, bool vis) : m_body(body)
    {
        if (vis)
            m_mesh = rabbit::cylinder_mesh(0.1, 1, 10, 10);
    }

    RigidBodyDrawer(RigidBody* body, rabbit::mesh<float> mesh) : m_body(body), m_mesh(mesh)
    {}

    void draw_on(rabbit::view& v) override
    {
        v.draw_mesh(m_mesh, rabbit::react_cast(m_body->getTransform()).to_mat4());
    }
};

void set_joint_torque(HingeJoint* joint, float torque)
{
    if (torque == 0) 
    {
        joint->enableMotor(false);
        joint->setMaxMotorTorque(0);
        joint->setMotorSpeed(0);
    }

    float sign = std::signbit(torque) ? -1 : 1;
    joint->enableMotor(true);
    joint->setMaxMotorTorque(std::abs(torque));
    joint->setMotorSpeed(1000000 * sign);
}

double joint_angular_velocity(HingeJoint* joint)
{
    return joint->getMotorSpeed();
}

class RigidBodyPair 
{
public:
    RigidBody* body;
    std::unique_ptr<RigidBodyDrawer> drawer;

public:
    RigidBodyPair() : body(nullptr) {}

    RigidBodyPair(RigidBody* body, bool vis) : body(body), drawer(new RigidBodyDrawer(body, vis))
    {}

    RigidBodyPair(RigidBodyPair&& oth) : body(oth.body), drawer(std::move(oth.drawer))
    {
        oth.body = nullptr;
    }

    RigidBodyPair& operator=(RigidBodyPair&& oth)
    {
        body = oth.body;
        drawer = std::move(oth.drawer);
        oth.body = nullptr;
        return *this;
    }

    static RigidBodyPair make(
        PhysicsWorld* world, 
        const ralgo::pose3<float>& pose, 
        bool vis = true)
    {
        auto body = world->createRigidBody(rabbit::react_cast(pose));
        return RigidBodyPair(body, vis);
    }

    ralgo::pose3<float> get_pose() const
    {
        return rabbit::react_cast(body->getTransform());
    }
};

HingeJoint * create_hinge_joint(
    PhysicsWorld* world,
    rabbit::vec3f point, 
    rabbit::vec3f _axis,
    RigidBody* body1,
    RigidBody* body2)
{
    const Vector3 anchorPoint = rabbit::react_cast(point);
    const Vector3 axis = rabbit::react_cast(_axis);
    HingeJointInfo jointInfo(body1, body2, anchorPoint, axis);
    HingeJoint * joint;
    joint = dynamic_cast<HingeJoint*>(world->createJoint(jointInfo)); 
    return joint;
}

FixedJoint * create_fixed_joint(
    PhysicsWorld* world,
    rabbit::vec3f point,
    RigidBody* body1,
    RigidBody* body2)
{
    const Vector3 anchorPoint = rabbit::react_cast(point);
    FixedJointInfo jointInfo(body1, body2, anchorPoint);
    FixedJoint * joint;
    joint = dynamic_cast<FixedJoint*>(world->createJoint(jointInfo)); 
    return joint;
}

class Manipulator 
{
public:
    PhysicsWorld* world;
    RigidBodyPair base_body;
    RigidBodyPair base_body_proxy;
    RigidBodyPair arm1;
    RigidBodyPair arm2;
    RigidBodyPair arm3;
    RigidBodyPair arm4;

    HingeJoint* joint0;
    HingeJoint* joint1;
    HingeJoint* joint2;
    HingeJoint* joint3;

    std::vector<RigidBodyPair*> bodies = {};
    std::vector<RigidBodyPair*> arms = {};
    std::vector<HingeJoint*> joints = {};

public:
    Manipulator(
        PhysicsWorld* world, 
        const ralgo::pose3<float>& initial_pose = ralgo::pose3<float>())
        : 
        world(world)
    {
        base_body = RigidBodyPair::make(
            world, initial_pose * ralgo::mov3<float>({0,0,0}), false);
        base_body_proxy = RigidBodyPair::make(
            world, initial_pose * ralgo::mov3<float>({0,0,0}), false);
        arm1 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,0}));
        arm2 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,1}));
        arm3 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,2}));
        //arm4 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,-4}));
        base_body.body->setType(BodyType::STATIC);

        create_fixed_joint(world, base_body.get_pose().lin, base_body_proxy.body, base_body.body);
        joint0 = create_hinge_joint(world, arm1.get_pose().lin, {0,1,0}, arm1.body, base_body_proxy.body);
        joint1 = create_hinge_joint(world, arm2.get_pose().lin, {0,1,0}, arm2.body, arm1.body);
        joint2 = create_hinge_joint(world, arm3.get_pose().lin, {0,1,0}, arm3.body, arm2.body);
        //joint3 = create_hinge_joint(world, arm3.get_pose().lin, {0,1,0}, arm4.body, arm3.body);

        bodies = {&base_body, &base_body_proxy, &arm1, &arm2, &arm3};//, &arm4};
        arms = {&arm1, &arm2, &arm3};
        joints = {joint0, joint1, joint2};//, joint3};

        create_colliders();
        
        for (auto* arm : arms)
        { 
            nos::println(arm->body->getMass());
            nos::println(rabbit::react_cast(arm->body->getLocalCenterOfMass()));
            nos::println(rabbit::react_cast(arm->body->getLocalInertiaTensor()));

            auto target = linalg::vec<float,3> {0,0,2};

            auto inertia1 = rabbit::react_world_inertia_for_center(*arm1.body, target);
            nos::println(inertia1);

            auto inertia2 = rabbit::react_world_inertia_for_center(*arm2.body, target);
            nos::println(inertia2);

            auto inertia3 = rabbit::react_world_inertia_for_center(*arm3.body, target);
            nos::println(inertia3);
        }
    }

    linalg::mat<float,3,3> arm1_inertia() 
    {
        auto target = arm1.get_pose().lin;
        auto inertia_arm1 = rabbit::react_world_inertia_for_center(*arm1.body, target);
        auto inertia_arm2 = rabbit::react_world_inertia_for_center(*arm2.body, target);
        auto inertia_arm3 = rabbit::react_world_inertia_for_center(*arm3.body, target);
        return inertia_arm1 + inertia_arm2 + inertia_arm3;
    }
    
    linalg::mat<float,3,3> arm2_inertia() 
    {
        auto target = arm2.get_pose().lin;
        auto inertia_arm2 = rabbit::react_world_inertia_for_center(*arm2.body, target);
        auto inertia_arm3 = rabbit::react_world_inertia_for_center(*arm3.body, target);
        return inertia_arm2 + inertia_arm3;
    }

    linalg::mat<float,3,3> arm3_inertia() 
    {
        auto target = arm3.get_pose().lin;
        auto inertia_arm3 = rabbit::react_world_inertia_for_center(*arm3.body, target);
        return inertia_arm3;
    }

    void add_collider_for_arm(RigidBodyPair& arm)
    {
        auto * shape = physicsCommon.createCapsuleShape(0.25, 0.4);
        auto transform = rabbit::react_cast(
            ralgo::mov3<float>({0,0,0.5}) * 
            ralgo::rot3<float>({1,0,0}, M_PI/2));
        arm.body->addCollider(shape, transform);
    }

    void create_colliders() 
    {
        for (auto* body : arms)
        {
            add_collider_for_arm(*body);
            body->body->updateMassPropertiesFromColliders();
        }
    }

    void bind_to_scene(rabbit::scene& scene)
    {
        for (auto& body : bodies)
        {
            scene.add_object(body->drawer.get());
        }
    }
};

int main() 
{
    rabbit::scene scene;
    auto window = rabbit::create_glfw_window();
    auto view = scene.create_view();
    view->enable_trihedron();

	PhysicsWorld* world = physicsCommon.createPhysicsWorld();
    world->setGravity(Vector3(0.0, 0.0, -1.0));

    Manipulator manipulator(world);
    manipulator.bind_to_scene(scene);

    const int N = 3;
    ralgo::servo_second_tier servo[N];

    //set_joint_torque(manipulator.joint0, 5);

    //double T_v = 0.1, ksi_v=0.75;
    double T_p = 0.1, ksi_p=2;

    for (int i = 0; i < N; ++i)
    {
    //    servo[i].setup_velocity_parameters(T_v, ksi_v, 0.33);
        servo[i].setup_position_parameters(T_p, ksi_p, 1);
    }

    double lastang[N] = {};
    while (!glfwWindowShouldClose(window)) 
    //while (true) 
    {
        /*view->camera.set_camera(
            rabbit::vec3f{10.f*cos(glfwGetTime()), 10.f*sin(glfwGetTime()), 3},
            {0, 0, 0}
        );*/
        view->camera.set_camera(
           rabbit::vec3f{0.f, 10.f, 3},
           {0, 0, 0}
        );

        double timeStep = 1.0f / 1000.0f;

        double target[N];
        auto loct = igris::millis() % 32000;
        if (loct < 4000)       { target[0] = 0; target[1] = 0;  target[2] = 0; }
        else if (loct < 8000)  { target[0] = 0; target[1] = 0;  target[2] = 1; }
        else if (loct < 12000)  { target[0] = 0; target[1] = 1;  target[2] = 0; }
        else if (loct < 16000)  { target[0] = 0; target[1] = 1; target[2] = 1; }
        else if (loct < 20000) { target[0] =  1; target[1] = 0;  target[2] = 0; }
        else if (loct < 24000) { target[0] =  1; target[1] = 0;  target[2] = 1; }
        else if (loct < 28000) { target[0] =  1; target[1] = 1;  target[2] = 0; }
        else                   { target[0] =  1; target[1] = 1; target[2] = 1; }

        //target[0] = 1; target[1] = -1;  target[2] = 1;

        auto inertia1 = manipulator.arm1_inertia();
        auto inertia2 = manipulator.arm2_inertia();
        auto inertia3 = manipulator.arm3_inertia();
        servo[0].setup_velocity_filter(inertia1[1][1]*30, 0);
        servo[1].setup_velocity_filter(inertia2[1][1]*30, 0);
        servo[2].setup_velocity_filter(inertia3[1][1]*30, 0);
        nos::println(inertia1[1][1]);
        nos::println(inertia2[1][1]);
        nos::println(inertia3[1][1]);
        
        for (int i = 0; i < N; ++i)
        {
            auto curang = -manipulator.joints[i]->getAngle(); 
            auto angvel = ralgo::angdiff<float>(curang, lastang[i]) / timeStep;
            auto torque = servo[i].mark_position_control(target[i], curang, angvel, 1, timeStep);
            set_joint_torque(manipulator.joints[i], torque);
            lastang[i] = curang;
        }

        world->update(timeStep);
        scene.update();
        
        glfwPollEvents();
        glfwSwapBuffers(window);
    }


}