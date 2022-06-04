#include <reactphysics3d/reactphysics3d.h>
#include <rabbit/opengl/qtscene.h>
#include <rabbit/opengl/window.h>
#include <rabbit/compat/reactphysics3d.h>
#include <rabbit/mesh/mesh.h> 
#include <rabbit/mesh/primitives.h> 

using namespace reactphysics3d;

class RigidBodyDrawer : public rabbit::drawable_object 
{ 
    RigidBody* m_body;
    rabbit::mesh<float> m_mesh;

public:
    RigidBodyDrawer(RigidBody* body) : m_body(body)
    {
        m_mesh = rabbit::sphere_mesh(0.1, 10, 10);
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

class RigidBodyPair 
{
public:
    RigidBody* body;
    std::unique_ptr<RigidBodyDrawer> drawer;

public:
    RigidBodyPair() : body(nullptr) {}

    RigidBodyPair(RigidBody* body) : body(body), drawer(new RigidBodyDrawer(body))
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

    static RigidBodyPair make(PhysicsWorld* world, const ralgo::pose3<float>& pose)
    {
        auto body = world->createRigidBody(rabbit::react_cast(pose));
        return RigidBodyPair(body);
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
    std::vector<HingeJoint*> joints = {};

public:
    Manipulator(PhysicsWorld* world, const ralgo::pose3<float>& initial_pose = ralgo::pose3<float>())
        : world(world)
    {
        base_body = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,0}));
        base_body_proxy = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,0}));
        arm1 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,-1}));
        arm2 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,-2}));
        arm3 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,-3}));
        arm4 = RigidBodyPair::make(world, initial_pose * ralgo::mov3<float>({0,0,-4}));
        base_body.body->setType(BodyType::STATIC);

        create_fixed_joint(world, base_body.get_pose().lin, base_body_proxy.body, base_body.body);
        joint0 = create_hinge_joint(world, base_body_proxy.get_pose().lin, {0,1,0}, arm1.body, base_body_proxy.body);
        joint1 = create_hinge_joint(world, arm1.get_pose().lin, {0,1,0}, arm2.body, arm1.body);
        joint2 = create_hinge_joint(world, arm2.get_pose().lin, {0,1,0}, arm3.body, arm2.body);
        joint3 = create_hinge_joint(world, arm3.get_pose().lin, {0,1,0}, arm4.body, arm3.body);

        bodies = {&base_body, &base_body_proxy, &arm1, &arm2, &arm3, &arm4};
        joints = {joint0, joint1, joint2, joint3};
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

    PhysicsCommon physicsCommon;
	PhysicsWorld* world = physicsCommon.createPhysicsWorld();
    world->setGravity(Vector3(0.0, 0.0, -1));

    Manipulator manipulator(world);
    manipulator.bind_to_scene(scene);

    set_joint_torque(manipulator.joint0, 5);

    while (!glfwWindowShouldClose(window)) 
    {
        /*view->camera.set_camera(
            rabbit::vec3f{10.f*cos(glfwGetTime()), 10.f*sin(glfwGetTime()), 3},
            {0, 0, 0}
        );*/
        view->camera.set_camera(
            rabbit::vec3f{0.f, 10.f, 3},
            {0, 0, 0}
        );

        double timeStep = 1.0f / 60.0f;

        world->update(timeStep);
        scene.update();

        nos::println(rabbit::react_cast(manipulator.joint0->getReactionTorque(timeStep)));

        glfwPollEvents();
        glfwSwapBuffers(window);
    }


}