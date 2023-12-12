from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package="gnss_2_utm",
         executable="gnss_2_utm",
         name="gnss_2_utm",
         output="screen",
         emulate_tty=True
      ),
   ])
