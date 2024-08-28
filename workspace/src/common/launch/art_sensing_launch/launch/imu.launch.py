#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#

# ros imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

# internal imports
from launch_utils import AddLaunchArgument, IncludeLaunchDescriptionWithCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    robot_ns = LaunchConfiguration('robot_ns')

    # ---------------
    # Launch Includes
    # ---------------
    package_share_directory = get_package_share_directory('art_sensing_launch')
    veh_config_file_path = PathJoinSubstitution([package_share_directory, 'config', LaunchConfiguration('veh_config_file')])

    node = Node(
        package="wheeltec_n100_imu",  
        executable="imu_node",
        name="wheeltec_imu_node",
        output="screen",
        parameters=[veh_config_file_path  
                      , {
                     'imu_frame': PythonExpression(['"', robot_ns, "/imu", '"']),
                     'imu_topic': PythonExpression(['"', '/', robot_ns, "/imu/data_raw", '"'])}],
        #remappings=[
        #    ('/imu_trueEast', PythonExpression(['"', '/', robot_ns, "/imu/data", '"']))
        #]
    )
    ld.add_action(node)


    imu_yaw_offset = Node(
       package="navsat_util",
       executable="imu_offset",
       name="automatic_yaw_calib",
       output="screen",
       remappings=[
           ('/imu/data_raw', PythonExpression(['"', '/', robot_ns, "/imu/data_raw", '"'])),
           ('/imu/data', PythonExpression(['"', '/', robot_ns, "/imu/data", '"']))
       ]
    )
    ld.add_action(imu_yaw_offset)

    return ld

