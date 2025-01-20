import numpy as np
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, Quaternion
# Make plt.show() responsive to ctrl+c
# import signal
# signal.signal(signal.SIGINT, signal.SIG_DFL)
# from lqr import LQR
from ilqr import iLQR
from diff_drive_model import DiffDrive

class Controller(Node):
    
    def __init__(self):
        super().__init__('ilqr_controller')
        self.odom_sub_ = self.create_subscription(Odometry, '/map_ekf/odometry', self.odom_callback, 10)
        self.path_pub_ = self.create_publisher(Path, 'ilqr_path', 10)

        self.x_bar_data = np.empty()
        self.y_bar_data = np.empty()
        self.tht_bar_data = np.empty()

    """
    Quaternion as wxyz
    """
    def quaternion_to_yaw_rad(self, quat: Quaternion):        
        return 2.0 * math.atan2(quat.z, quat.w)
    
    def odom_callback(self, msg: Odometry):
        state = msg.pose.pose
        self.update_path(state)

        self.path_pub_.publish()


    def update_path(self, state: Pose):
        
        """
        Declare constants and state variables
        """
        run_dynamics = False
        Ts = 0.1
        t_total = 10.0
        t_range = np.arange(0, t_total, Ts)
        # States. vl_wheel and vr_wheel are linear velocities of wheels
        n = np.size(np.array(['x', 'y','tht', 'vl_wheel', 'vr_wheel'])) # states
        m = np.size(np.array(['l_volts', 'r_volts'])) # states

        # Initial State
        x_0 = state.position.x
        y_0 = state.position.y
        tht_0 = self.quaternion_to_yaw_rad(state.orientation)
        vr_wheel_0 = 0.0
        vl_wheel_0 = 0.0
        l_volts_0 = 0.0
        r_volts_0 = 0.0

        X_0 = np.array([x_0, y_0, tht_0, vl_wheel_0, vr_wheel_0])
        U_0 = np.array([l_volts_0, r_volts_0])


        """
        Create Differential Drive model
        """
        drive = DiffDrive(Ts, n, m)


        """
        Initialize iLQR
        """
        x_exit = 1e-3
        ilqr = iLQR(Ts, t_total, drive, X_0, U_0)


        """
        Get iLQR initial guess
        """
        ilqr.initial_forward_rollout()


        """
        Run initial backward/forward pass
        """
        del_u_star_data, V_star_data = ilqr.back_pass(ilqr.x_0_data, ilqr.u_0_data)
        x_data, u_data = ilqr.for_pass(ilqr.x_0_data, list(reversed(del_u_star_data.values())), ilqr.u_0_data)
        old_x_data = ilqr.x_0_data
        old_u_data = ilqr.u_0_data


        """
        Calculate difference between initial trajectory and latest trajectory for passes
        """
        x_diff = sum(x_data[0, :] - old_x_data[0,:])
        y_diff = sum(x_data[1, :] - old_x_data[1,:])
        tht_diff = sum(x_data[2, :] - old_x_data[2,:])
        vl_diff = sum(x_data[3, :] - old_x_data[3,:])
        vr_diff = sum(x_data[4, :] - old_x_data[4,:])


        """
        Get scalar difference between old and new trajectories
        """
        mag_diff = x_diff + y_diff + tht_diff + vl_diff + vr_diff

        old_x_data = x_data
        old_u_data = u_data

        
        """
        Loop until a more optimal trajectory is found
        """
        while mag_diff <= x_exit:
            ilqr = iLQR(Ts, t_total, drive, old_x_data, old_u_data)
            ilqr.get_linear_dyn()
            del_u_star_data, V_star_data = ilqr.back_pass(old_x_data, old_u_data)
            x_data, u_data = ilqr.for_pass(old_x_data, list(reversed(del_u_star_data.values())), old_u_data)

            # Calculate difference between trajectories
            x_diff = sum(x_data[0, :] - old_x_data[0,:])
            y_diff = sum(x_data[1, :] - old_x_data[1,:])
            tht_diff = sum(x_data[2, :] - old_x_data[2,:])
            vl_diff = sum(x_data[3, :] - old_x_data[3,:])
            vr_diff = sum(x_data[4, :] - old_x_data[4,:])

            mag_diff = x_diff + y_diff + tht_diff + vl_diff + vr_diff

            old_x_data = x_data
            old_u_data = u_data

            self.x_bar_data = x_data[0, :]
            self.y_bar_data = x_data[1, :]
            self.tht_bar_data = x_data[2, :]
            # vl_bar_data = x_data[3, :]
            # vr_bar_data = x_data[4, :]

        if run_dynamics:
            data = np.zeros(shape = (n, np.size(t_range)))
            x_bar_data = []
            y_bar_data = []
            tht_bar_data = []
            
            # Dynamics Rollout
            for i in range(np.size(t_range)):
                if i == 0:
                    x = (drive.next_step(X_0, U_0))
                    x_bar_data.append(x[0])
                    y_bar_data.append(x[1])
                    tht_bar_data.append(x[2])
                else:
                    x = (drive.next_step(data[:, i - 1], U_0))
                    x_bar_data.append(x[0])
                    y_bar_data.append(x[1])
                    tht_bar_data.append(x[2])
                data[:, i] = x


def main(args=None):
    """
    Spin controller node
    """
    controller_ = Controller()
    rclpy.spin(controller_)
    controller_.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()