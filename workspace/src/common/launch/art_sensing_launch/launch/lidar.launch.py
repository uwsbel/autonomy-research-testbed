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

# internal imports
from launch_utils import AddLaunchArgument, IncludeLaunchDescriptionWithCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration(
        "serial_baudrate", default="1000000"
    )  # for s2 is 1000000
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="DenseBoost")
    # ---------------
    # Launch Includes
    # ---------------

    node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": channel_type,
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "scan_mode": scan_mode,
            }
        ],
        output="screen",
    )
    ld.add_action(node)

    return ld
