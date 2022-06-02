#include <reactphysics3d/reactphysics3d.h>
#include <rabbit/opengl/qtscene.h>
#include <rabbit/opengl/window.h>

using namespace reactphysics3d;

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
        rabbit::vec3f{ 0.3f, 0.7f, 0.0f }, 
        rabbit::vec3f{ 0.0f, 0.0f, 0.0f }
    });

    scene.add_object(&polyline);

    while (!glfwWindowShouldClose(window)) 
    {
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