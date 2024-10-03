#!/usr/bin/env python3
import rospy
import tf
import tf.transformations

# Initialize the TF broadcaster and listener
rospy.init_node('position_estimator', anonymous=True)

tf_broadcaster = tf.TransformBroadcaster()
tf_listener = tf.TransformListener()

# Initialize variables to store last transformation values
last_map_to_AMCL_translation = None
last_map_to_AMCL_quaternion = None

# Function to broadcast the transformation
def broadcast_map_to_odom():
    global last_map_to_AMCL_translation, last_map_to_AMCL_quaternion

    try:
        # Step 1: Get the current odom -> base_link transformation
        tf_listener.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(1.0))
        (odom_to_base_translation, odom_to_base_quaternion) = tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))

        # Convert odom_to_base_quaternion to Euler angles to work with the yaw
        _, _, odom_to_base_yaw = tf.transformations.euler_from_quaternion(odom_to_base_quaternion)

        # Step 2: Get the map -> AMCL transformation
        tf_listener.waitForTransform("map", "AMCL", rospy.Time(0), rospy.Duration(5.0))
        (map_to_AMCL_translation, map_to_AMCL_quaternion) = tf_listener.lookupTransform("map", "AMCL", rospy.Time(0))

        # Check if the transformation has changed
        if (last_map_to_AMCL_translation != map_to_AMCL_translation or
                last_map_to_AMCL_quaternion != map_to_AMCL_quaternion):
            
            # Update last transformation values
            last_map_to_AMCL_translation = map_to_AMCL_translation
            last_map_to_AMCL_quaternion = map_to_AMCL_quaternion

            # Convert map_to_base_quaternion to Euler angles for yaw
            _, _, map_to_AMCL_yaw = tf.transformations.euler_from_quaternion(map_to_AMCL_quaternion)

            # Step 3: Compute the map -> odom transformation
            # Translation difference: map_to_odom_translation = map_to_base_translation - odom_to_base_translation
            map_to_odom_translation = (
                map_to_AMCL_translation[0] - odom_to_base_translation[0],
                map_to_AMCL_translation[1] - odom_to_base_translation[1],
                0  # Assuming 2D
            )

            # Rotation difference: map_to_odom_yaw = map_to_base_yaw - odom_to_base_yaw
            map_to_odom_yaw = map_to_AMCL_yaw - odom_to_base_yaw
            map_to_odom_quaternion = tf.transformations.quaternion_from_euler(0, 0, map_to_odom_yaw)

            # Step 4: Broadcast the transformation from map to odom
            tf_broadcaster.sendTransform(
                map_to_odom_translation,
                map_to_odom_quaternion,
                rospy.Time.now(),
                "odom",  # Child frame
                "map"    # Parent frame
            )

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to lookup transformations")

# Main loop to continuously broadcast the transformation
if __name__ == "__main__":
    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        broadcast_map_to_odom()
        rate.sleep()
