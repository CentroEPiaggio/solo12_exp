from dataclasses import dataclass
import logging
import os

from cycler import cycler
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import progressbar
import quaternion

from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.duration
from rclpy.serialization import deserialize_message
import ros2_numpy as rnp
import rosbag2_py

from mocap_msgs.msg import RigidBodies
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from bag_analyzer.utils import interp, quat2rpy, quat_rot



class BagAnalyzer():
    """Class than analyze all the bags file, plot their data and compute the KPIs."""
    
    @dataclass
    class BagData:
        """Container for the data extracted from the bag file."""
        
        time_joint_states: np.ndarray
        joint_pos: np.ndarray
        torque: np.ndarray
        time_mocap: np.ndarray
        position: np.ndarray
        orientation: np.ndarray
        rpy: np.ndarray
        lin_velocity: np.ndarray
        time_commands: np.ndarray
        joint_pos_command: np.ndarray
    
    def __init__(self) -> None:
        package_share_directory = get_package_share_directory('bag_analyzer')
        self.bags_directory = f"{package_share_directory}/../../../../bags"
        
        # Order with thich the joint data is stored.
        self.joint_names = [
            "LF_HAA", "LF_HFE", "LF_KFE",
            "RF_HAA", "RF_HFE", "RF_KFE",
            "LH_HAA", "LH_HFE", "LH_KFE",
            "RH_HAA", "RH_HFE", "RH_KFE",
        ]
        
        self.kpi_df = pd.DataFrame(columns=[
            "Bag Name", "Heading RMSE", "CoT", "Yaw Drift"
        ])
        
        np.seterr(divide='ignore', invalid='ignore')
        
        self._plot_init()
    
    def __call__(self):
        bags = self.list_bag_files(self.bags_directory)

        progressbar.streams.wrap_stderr()
        logging.basicConfig()
        for i in progressbar.progressbar(range(len(bags))):
            bag = bags[i]
            try:
                self._analyze_bag(bag)
            except Exception as e:
                logging.error("Error while analyzing the bag %s.", bag)
                # logging.error(e)
                
        self._compute_mean_kpi()
    
    @staticmethod
    def list_bag_files(directory: str) -> list[str]:
        """Get a list of all the bag files in the input directory and its subdirectories."""
        
        bag_files = []
        for root, _, files in os.walk(directory):
            for file in files:
                if file.endswith('.mcap'):
                    bag_files.append(os.path.join(root, file))
        return bag_files
    
    def _analyze_bag(self, bag_file_path: str):
        """Analyze the data in the input bag file, plot the data, and save the KPIs."""
        
        data = self._extract_data(bag_file_path)
        
        self._plot_data(data, bag_file_path)
        
        if not os.path.exists(f'{self.bags_directory}/csv'):
            os.makedirs(f'{self.bags_directory}/csv', exist_ok=True)
        self._compute_kpi(data, bag_file_path.split('/')[-2])
        self.kpi_df.to_csv(f"{self.bags_directory}/csv/kpi.csv", index=False)
    
    def _extract_data(self, bag_file_path: str):
        # ========================== Initialization ========================== #
        
        # Create the bag reader.
        reader = rosbag2_py.SequentialReader()
        storage_options: rosbag2_py.StorageOptions = rosbag2_py._storage.StorageOptions(
            uri=bag_file_path, storage_id='mcap')
        converter_options: rosbag2_py.ConverterOptions = rosbag2_py._storage.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        
        # Initialize the arrays.
        exp_started = False
        
        time = 0
        final_time = 0
        
        time_joint_states = np.array([])
        joint_pos = np.zeros((0, 12))
        torque = np.zeros((0, 12))
        
        time_mocap = np.array([])
        position = np.zeros((0, 3))
        orientation = np.array([])
        
        time_commands = np.array([])
        joint_pos_command = np.zeros((0, 12))
        
        # ================== Read The Bag And Save The Data. ================= #
        
        while reader.has_next():
            topic, data, _ = reader.read_next()
            
            if topic == '/clock':
                time_msg: Clock = deserialize_message(data, Clock)
                time = time_msg.clock.sec + time_msg.clock.nanosec * 1e-9
            
            # The experiment starts when the robot is commanded to raise its feet.
            # This happens when at least one of the four optimal forces is zero (along all three components).
            if not exp_started and topic == '/logging/optimal_forces':
                optimal_forces: Float64MultiArray = deserialize_message(data, Float64MultiArray)
                for i in range(4):
                    if optimal_forces.data[i] == 0 and optimal_forces.data[i+1] == 0 and optimal_forces.data[i+2] == 0:
                        exp_started = True
            
            if not exp_started:
                continue
            
            if topic == '/logging/optimal_forces':
                optimal_forces: Float64MultiArray = deserialize_message(data, Float64MultiArray)
                for i in range(4):
                    if optimal_forces.data[i] == 0 and optimal_forces.data[i+1] == 0 and optimal_forces.data[i+2] == 0:
                        final_time = time
                
            # Save the motion capture data.
            if topic == '/rigid_bodies':
                rigid_body_msg: RigidBodies = deserialize_message(data, RigidBodies)
                for rigid_body in rigid_body_msg.rigidbodies:
                    if rigid_body.rigid_body_name == 'SOLO12':
                        time_mocap = np.concatenate((time_mocap, [time]))
                        
                        q = rigid_body.pose.orientation
                        quat = np.quaternion(q.w, q.x, q.y, q.z)
                        orientation = np.concatenate((orientation, [quat]))
                        
                        pos = quat_rot(quat.conj(), rnp.numpify(rigid_body.pose.position))
                        position = np.vstack((position, pos))
            
            # Save the measured joint positions and torques.
            if topic == '/joint_states':
                joint_state_msg: JointState = deserialize_message(data, JointState)
                time_joint_states = np.concatenate((time_joint_states, [time]))
                pos_reordered = np.array(joint_state_msg.position)
                torques_reordered = np.array(joint_state_msg.effort)
                
                for i, joint_name in enumerate(joint_state_msg.name):
                    pos_reordered[self.joint_names.index(joint_name)] = joint_state_msg.position[i]
                    torques_reordered[self.joint_names.index(joint_name)] = joint_state_msg.effort[i]
                
                joint_pos = np.vstack((joint_pos, pos_reordered))
                torque = np.vstack((torque, torques_reordered))
                
            # Save the reference joint positions and torques.
            if topic == '/PD_control/command':
                joint_state_msg: JointState = deserialize_message(data, JointState)
                pos_reordered = np.array(joint_state_msg.position)
                
                for i, joint_name in enumerate(joint_state_msg.name):
                    joint_name = joint_name[1] + joint_name[0] + joint_name[2:]
                    pos_reordered[self.joint_names.index(joint_name)] = joint_state_msg.position[i]
                
                time_commands = np.concatenate((time_commands, [time]))
                joint_pos_command = np.vstack((joint_pos_command, pos_reordered))
                
        # ============ Eliminate The Final Part Of The Experiment. =========== #
        
        # The final instants of the experiment need to be removed.
                
        torque = torque[(time_joint_states <= final_time), :]
        joint_pos = joint_pos[(time_joint_states <= final_time), :]
        time_joint_states = time_joint_states[(time_joint_states <= final_time)]
        
        time_joint_states = np.unique(time_joint_states)
        torque = torque[0:len(time_joint_states), :]
        joint_pos = joint_pos[0:len(time_joint_states), :]
        
        
        position = position[time_mocap <= final_time, :]
        orientation = orientation[time_mocap <= final_time]
        time_mocap = time_mocap[time_mocap <= final_time]
        
        time_mocap = np.unique(time_mocap)
        position = position[0:len(time_mocap), :]
        orientation = orientation[0:len(time_mocap)]
        
        
        joint_pos_command = joint_pos_command[time_commands <= final_time, :]
        time_commands = time_commands[time_commands <= final_time]
        
        time_commands = np.unique(time_commands)
        joint_pos_command = joint_pos_command[0:len(time_commands), :]
        
        # ==================================================================== #
        
        # Compute the linear velocity by differentiating the position.
        lin_velocity = np.vstack(
            [np.gradient(position[:, i], time_mocap) for i in range(3)]
        ).transpose()
        
        rpy = quat2rpy(orientation)
        
        return self.BagData(
            time_joint_states=time_joint_states,
            joint_pos=joint_pos,
            torque=torque,
            time_mocap=time_mocap,
            position=position,
            orientation=orientation,
            rpy=rpy,
            lin_velocity=lin_velocity,
            time_commands=time_commands,
            joint_pos_command=joint_pos_command,
        )
        
    @staticmethod
    def _plot_init():
        """Set the default plot style."""
        
        default_cycler = (
            cycler(color=['#0072BD', '#D95319', '#EDB120', '#7E2F8E']) +
            cycler('linestyle', ['-', '--', '-', '--'])
        )

        textsize = 12
        labelsize = 16

        # plt.rc('font', family='serif', serif='Times')
        # plt.rc('text', usetex=True)
        plt.rc('xtick', labelsize=textsize)
        plt.rc('ytick', labelsize=textsize)
        plt.rc('axes', labelsize=labelsize, prop_cycle=default_cycler)
        plt.rc('legend', fontsize=textsize)

        plt.rc("axes", grid=True)
        plt.rc("grid", linestyle='dotted', linewidth=0.25)

        plt.rcParams['figure.constrained_layout.use'] = True
    
    def _plot_data(self, data: BagData, bag_file_path: str):
        """Plot the data and save the figures."""
        
        [x_size_def, y_size_def] = plt.rcParams.get('figure.figsize')
        
        if not os.path.exists(f'{self.bags_directory}/plots'):
            os.makedirs(f'{self.bags_directory}/plots', exist_ok=True)
        
        # Create the directory where the plots will be saved
        fig_path = os.path.join(
            os.sep, *bag_file_path.replace(self.bags_directory, f"{self.bags_directory}/plots").split('/')[:-1]
        )
        os.makedirs(fig_path, exist_ok=True)
        
        # ============= Position, Velocity, Orientation, Torques. ============ #
        
        fig, axs = plt.subplots(2, 2, figsize=[2*x_size_def, 2*y_size_def], layout="constrained")
        
        for ax1 in axs:
            for ax in ax1:
                ax.autoscale(enable=True, axis='x', tight=True)
                
        axs[0, 0].plot(data.time_mocap - data.time_joint_states[0], data.position)
        axs[0, 0].set(
            xlabel='Time [s]',
            ylabel='Position [m]',
            title='Base Position'
        )
        axs[0, 0].legend(['x-axis', 'y-axis', 'z-axis'])
        
        axs[0, 1].plot(data.time_mocap - data.time_joint_states[0], data.lin_velocity)
        axs[0, 1].set(
            xlabel='Time [s]',
            ylabel='Velocity [m/s]',
            title='Base Velocity'
        )
        axs[0, 1].legend(['x-axis', 'y-axis', 'z-axis'])
        
        axs[1, 1].plot(data.time_joint_states - data.time_joint_states[0], data.torque)
        axs[1, 1].set(
            xlabel='Time [s]',
            ylabel='Torque [N m]',
            title='Joint Torques'
        )
            
        plt.savefig(f"{fig_path}/mocap.svg", bbox_inches="tight", format='svg')
        plt.close(fig)
        
        # ======================= Joint Position Errors ====================== #
        
        fig, axs = plt.subplots(2, 2, figsize=[2*x_size_def, 2*y_size_def], layout="constrained")
        
        for ax1 in axs:
            for ax in ax1:
                ax.autoscale(enable=True, axis='x', tight=True)
        
        time = np.linspace(data.time_commands[0], data.time_commands[-1], 1000)
        
        joint_pos = interp(time, data.time_joint_states, data.joint_pos)
        joint_pos_command = interp(time, data.time_commands, data.joint_pos_command)
        
        titles = [
            "LF Leg Position Errors", "RF Leg Position Errors",
            "LH Leg Position Errors", "RH Leg Position Errors"
        ]
        
        for i in range(4):
            axs[i//2, i%2].plot(
                time,
                (joint_pos[:, 3*i:3*(i+1)] - joint_pos_command[:, 3*i:3*(i+1)]) * 180 / np.pi
            )
            axs[i//2, i%2].set(
                xlabel='Time [s]',
                ylabel='Position Error [deg]',
                title=titles[i]
            )
            axs[i//2, i%2].legend(['HAA', 'HFE', 'KFE'])
            
        plt.savefig(f"{fig_path}/joint_pos.svg", bbox_inches="tight", format='svg')
            
        
    def _compute_kpi(self, data: BagData, bag_filename: str):
        vel_ref = 0.1
        
        heading_rmse = np.sqrt(np.mean(
            (data.lin_velocity[:, 0] - vel_ref)**2
        ))
        
        # CoT = (d_traveled / time) / (mean(torque^2))
        cot = ((data.position[-1, 0] - data.position[0, 0]) / data.time_mocap[-1]) \
            / np.mean(np.sum(data.torque**2, axis=1)**0.5)
        
        yaw_drift = min(
            np.abs(data.rpy[-1, 2] - data.rpy[0, 2]),
            2*np.pi - np.abs(data.rpy[-1, 2] - data.rpy[0, 2])
        )
        
        self.kpi_df = pd.concat([
            self.kpi_df, pd.DataFrame({
                "Bag Name": [bag_filename],
                "Heading RMSE": [heading_rmse],
                "CoT": [cot],
                "Yaw Drift": [yaw_drift]
            })
        ])

    def _compute_mean_kpi(self):
        """Compute the mean KPI over the repeated experiments."""
        
        df = self.kpi_df
        
        df['Bag Name'] = df['Bag Name'].apply(lambda s: s[20:])
        
        mean_kpi_df = df.groupby('Bag Name').mean().reset_index()
        
        mean_kpi_df.to_csv(f"{self.bags_directory}/csv/mean_kpi.csv", index=False)
    
    

def main(args=None):
    rclpy.init(args=args)
        
    inspector = BagAnalyzer()
    inspector()
        
    rclpy.shutdown()


if __name__ == '__main__':
    main()
