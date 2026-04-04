#!/usr/bin/env python3
### HJ : 3D version of ot_sector_slicing.py — matplotlib 3D with slider/button
import rospy, rospkg
import yaml, os, subprocess, time
from f110_msgs.msg import WpntArray
from visualization_msgs.msg import MarkerArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D

class OvertakingSectorSlicer:
    def __init__(self):
        rospy.init_node('ot_sector_node', anonymous=True)

        self.glb_wpnts = None
        self.glb_sp_wpnts = None
        self.track_bounds = None

        self.glob_slider_s = 0
        self.sector_pnts = [0]

        rospy.Subscriber('/global_waypoints', WpntArray, self.glb_wpnts_cb)
        rospy.Subscriber('/global_waypoints/shortest_path', WpntArray, self.glb_sp_wpnts_cb)
        rospy.Subscriber('/trackbounds/markers', MarkerArray, self.bounds_cb)

        self.yaml_dir = rospy.get_param('~save_dir')

    def glb_wpnts_cb(self, data):
        self.glb_wpnts = data

    def glb_sp_wpnts_cb(self, data):
        self.glb_sp_wpnts = data

    def bounds_cb(self, data):
        self.track_bounds = data

    def slice_loop(self):
        print('Waiting for global waypoints...')
        rospy.wait_for_message('/global_waypoints', WpntArray)
        rospy.wait_for_message('/global_waypoints/shortest_path', WpntArray)

        self.sector_gui()
        print('Selected Overtaking Sector IDXs:', self.sector_pnts)

        self.sectors_to_yaml()

    def sector_gui(self):
        x = np.array([w.x_m for w in self.glb_wpnts.wpnts])
        y = np.array([w.y_m for w in self.glb_wpnts.wpnts])
        z = np.array([w.z_m for w in self.glb_wpnts.wpnts])
        s = np.array([w.s_m for w in self.glb_wpnts.wpnts])

        x_sp = np.array([w.x_m for w in self.glb_sp_wpnts.wpnts])
        y_sp = np.array([w.y_m for w in self.glb_sp_wpnts.wpnts])
        z_sp = np.array([w.z_m for w in self.glb_sp_wpnts.wpnts])

        bnd_x = np.array([m.pose.position.x for m in self.track_bounds.markers])
        bnd_y = np.array([m.pose.position.y for m in self.track_bounds.markers])
        bnd_z = np.array([m.pose.position.z for m in self.track_bounds.markers])

        fig = plt.figure(figsize=(12, 10))
        ax1 = fig.add_axes([0.05, 0.25, 0.9, 0.7], projection='3d')
        axslider = fig.add_axes([0.15, 0.15, 0.7, 0.03])
        axselect = fig.add_axes([0.15, 0.08, 0.3, 0.05])
        axfinish = fig.add_axes([0.55, 0.08, 0.3, 0.05])

        def update_map(cur_s):
            ax1.cla()
            ax1.plot(x, y, z, 'b-', linewidth=0.7, label='IQP')
            ax1.plot(x_sp, y_sp, z_sp, 'r-', linewidth=0.7, label='SP')
            ax1.plot(bnd_x, bnd_y, bnd_z, 'g-', linewidth=0.4)
            ax1.scatter(x[cur_s], y[cur_s], z[cur_s], c='blue', s=50, zorder=10)
            if len(self.sector_pnts) > 0:
                ax1.scatter(x[self.sector_pnts], y[self.sector_pnts], z[self.sector_pnts],
                            c='red', s=50, zorder=10)
            ax1.set_xlabel('x [m]')
            ax1.set_ylabel('y [m]')
            ax1.set_zlabel('z [m]')
            ax1.set_title('OT Sector Slicing (idx=%d, s=%.1fm)' % (cur_s, s[cur_s]))
            ax1.view_init(elev=90, azim=-90)
            ax1.legend(fontsize=7)

        update_map(0)

        def update_s(val):
            idx = int(slider.val)
            if idx >= len(s):
                idx = len(s) - 1
            self.glob_slider_s = idx
            update_map(cur_s=idx)
            fig.canvas.draw_idle()

        def select_s(event):
            self.sector_pnts.append(self.glob_slider_s)
            update_map(cur_s=self.glob_slider_s)
            fig.canvas.draw_idle()

        def finish(event):
            plt.close()
            self.sector_pnts.append(len(s))
            self.sector_pnts = sorted(list(set(self.sector_pnts)))

        slider = Slider(axslider, 'Waypoint idx', 0, len(s)-1, valinit=0, valfmt='%d')
        slider.on_changed(update_s)

        btn_select = Button(axselect, 'Select OT S')
        btn_select.on_clicked(select_s)

        btn_finish = Button(axfinish, 'Done')
        btn_finish.on_clicked(finish)

        plt.show()

    def sectors_to_yaml(self):
        if len(self.sector_pnts) == 1:
            self.sector_pnts.append(len(self.glb_wpnts.wpnts))

        n_sectors = len(self.sector_pnts) - 1
        dict_file = {
            'n_sectors': n_sectors,
            'yeet_factor': 1.25,
            'spline_len': 30,
            'ot_sector_begin': 0.5
        }
        for i in range(0, n_sectors):
            dict_file['Overtaking_sector' + str(i)] = {
                'start': self.sector_pnts[i] if i == 0 else self.sector_pnts[i] + 1,
                'end': self.sector_pnts[i+1]}
            dict_file['Overtaking_sector' + str(i)].update({'ot_flag': False})

        yaml_path = os.path.join(self.yaml_dir, 'ot_sectors.yaml')
        with open(yaml_path, 'w') as file:
            print('Dumping to {}: {}'.format(yaml_path, dict_file))
            yaml.dump(dict_file, file, sort_keys=False)

        ros_path = rospkg.RosPack().get_path('overtaking_sector_tuner_3d')
        yaml_path = os.path.join(ros_path, 'cfg/ot_sectors.yaml')
        with open(yaml_path, 'w') as file:
            print('Dumping to {}: {}'.format(yaml_path, dict_file))
            yaml.dump(dict_file, file, sort_keys=False)

        time.sleep(1)
        print('Building overtaking_sector_tuner_3d...')
        shell_dir = os.path.join(ros_path, 'scripts/finish_sector.sh')
        if os.path.exists(shell_dir):
            subprocess.Popen(shell_dir, shell=True)

if __name__ == "__main__":
    ot_sector_slicer = OvertakingSectorSlicer()
    ot_sector_slicer.slice_loop()
