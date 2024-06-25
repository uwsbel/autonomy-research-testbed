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
from launch.substitutions import LaunchConfiguration, PythonExpression
# internal imports
from launch_utils import AddLaunchArgument, IncludeLaunchDescriptionWithCondition


def generate_launch_description():
    ld = LaunchDescription()
    robot_ns = LaunchConfiguration("robot_ns")

    # ---------------
    # Launch Includes
    # ---------------

    nmea_navsat_driver_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="nmea_serial_driver",
        output="screen",
        parameters=[{'port':'/dev/ttyACM0',
                     'frame_id': PythonExpression(['"', '/', robot_ns, "/gps", '"']),
            }],
        remappings=[('/fix', PythonExpression(['"','/',robot_ns,"/gps/fix",'"']))]
    )
    ld.add_action(nmea_navsat_driver_node)

    return ld

