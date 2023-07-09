
# yellow_light_sdf=$HOME/.gazebo/models/yellow_light/model.sdf
# ros2 run self_driving_car_pkg spawner_node $red_light_sdf red_light 0.0 0.0

'''
This Python script is used to spawn Gazebo models in a ROS2 environment.
 It takes the path of an SDF model file and the name of the model as arguments,
 and can optionally take the X, Y, and Z coordinates of the initial position of the model.
   The script creates a ROS2 node,
 connects to the Gazebo spawn_entity service, and sends a request to spawn the
 specified model at the specified position.
'''
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
def main():
    argv = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("Spawning_Node")
    client = node.create_client(SpawnEntity, "/spawn_entity")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("conencted to spawner")
    sdf_path = argv[0]
    request = SpawnEntity.Request()
    request.name = argv[1]
    # Use user defined positions (If provided)
    if len(argv)>3:
        request.initial_pose.position.x = float(argv[2])
        request.initial_pose.position.y = float(argv[3])
        if (argv[1] == 'beer'):
            request.initial_pose.position.z=1.5
    request.xml = open(sdf_path, 'r').read()

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()



# import sys
# import rclpy
# from gazebo_msgs.srv import SpawnEntity


# def main():
#     argv = sys.argv[1:]
#     rclpy.init()
#     node = rclpy.create_node("Spawning_Node")
#     client = node.create_client(SpawnEntity, "/spawn_entity")
#     if not client.service_is_ready():
#         client.wait_for_service()
#         node.get_logger().info("connected to spawner")

#     sdf_path = argv[0]
#     entity_name = argv[1]

#     # Check if the entity already exists
#     exists_request = SpawnEntity.Request()
#     exists_request.name = entity_name
#     future_exists = client.call_async(exists_request)
#     rclpy.spin_until_future_complete(node, future_exists)

#     if future_exists.result() is not None:
#         if future_exists.result().success:
#             node.get_logger().info("Entity [%s] already exists. Skipping spawn." % entity_name)
#             return
#         else:
#             raise RuntimeError("Failed to check if entity exists: %s" % future_exists.result().status_message)

#     spawn_request = SpawnEntity.Request()
#     spawn_request.name = entity_name
#     spawn_request.xml = open(sdf_path, 'r').read()

#     if len(argv) > 3:
#         spawn_request.initial_pose.position.x = float(argv[2])
#         spawn_request.initial_pose.position.y = float(argv[3])
#         if argv[1] == 'beer':
#             spawn_request.initial_pose.position.z = 1.5

#     node.get_logger().info("Sending service request to `/spawn_entity`")
#     future_spawn = client.call_async(spawn_request)
#     rclpy.spin_until_future_complete(node, future_spawn)

#     if future_spawn.result() is not None:
#         if future_spawn.result().success:
#             node.get_logger().info("Spawned entity [%s]" % entity_name)
#         else:
#             raise RuntimeError("Failed to spawn entity: %s" % future_spawn.result().status_message)
#     else:
#         raise RuntimeError("Failed to call service: spawn_entity")

#     node.get_logger().info("Done! Shutting down node.")
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()