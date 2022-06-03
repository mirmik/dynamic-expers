#include <reactphysics3d/reactphysics3d.h>
#include <rabbit/opengl/qtscene.h>
#include <rabbit/opengl/window.h>

using namespace reactphysics3d;

class Manipulator 
{
    PhysicsWorld* world;
    RigidBody* body0;
    RigidBody* body1;

    Manipulator(PhysicsWorld* world) 
        : world(world)
    {}
};

int main() 
{
    rabbit::scene scene;
    auto window = rabbit::create_glfw_window();
    auto view = scene.create_view();

    PhysicsCommon physicsCommon;
	PhysicsWorld* world = physicsCommon.createPhysicsWorld();

    // New position and orientation of the collision body
    Vector3 position (0.0 , 0.0 , 0.0) ;
    Quaternion orientation = Quaternion :: identity () ;
    Transform newTransform ( position , orientation ) ;
    RigidBody* body = world->createRigidBody(newTransform);

    world->setGravity(Vector3(0.0, 0.0, 0.0));

    rabbit::polyline_drawable_object polyline({
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, 0.0}
    });

    //view->camera.set_eye(rabbit::vec3f(100.0, 100.0, 100.0));
    //view->camera.set_target({0, 0, 0});

    scene.add_object(&polyline);

    while (!glfwWindowShouldClose(window)) 
    {
        view->camera.set_camera(
            rabbit::vec3f{10.f*cos(glfwGetTime()), 10.f*sin(glfwGetTime()), 3},
            {0, 0, 0}
        );
        
        world->update(1.0f / 60.0f);
        scene.update();

        // Get the new position and orientation of the collision body
        Transform transform = body->getTransform();

        // Print the new position and orientation of the collision body
        printf("Position: %f %f %f\n", transform.getPosition().x, transform.getPosition().y, transform.getPosition().z);
    

        glfwPollEvents();
        glfwSwapBuffers(window);
    }


}