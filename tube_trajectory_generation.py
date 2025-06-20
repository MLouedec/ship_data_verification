# generate the estimation tubes of the ship's trajectory
# one tube comes from the ship sensor platform [x_hat_a](.)
# the other tube comes from the land platform [x_hat_b](.)
# x(t) = [px(t),py(t)]
# px is the position of the boat on the x axis of the land platform (~distance between the boat and the land platform)
# py is the position of the boat on the y axis of the land platform

# these tubes are generating on two docking maneuvers (docking1 and docking2)

from lib import *

# -----------------------------------
# load the trajectory data files
# -----------------------------------

data_boat = pd.read_csv('datasets/boat_trajectory_from_ship_perspective.csv')
data_stereo = pd.read_csv('datasets/boat_trajectory_from_land_perspective.csv')

# synchronize time
data_stereo['time'] = data_stereo['time_sec']+data_stereo['time_nsec']*pow(10,-9)
t0 = min(data_boat['time'].iloc[0],data_stereo['time'].iloc[0])
data_boat['time'] = data_boat['time'] - t0
data_stereo['time'] = data_stereo['time'] - t0

print("initial time is t0 =",t0)

# small baseline correction on the sensor y of the stereo data

data_stereo['sensor_y'] = data_stereo['sensor_y'] + 0.5 * baseline

# extract the 2 docking maneuvers, docking1 [t1,t1_bis], docking2 [t2,t2_bis]
t1 = 1745835150 -t0
t1_bis = 1745835437 -t0
t2 = 1745836340 -t0
t2_bis = 1745836700 -t0
L_ti = np.array([[t1,t1_bis],[t2,t2_bis]])

docking1_boat = data_boat[(data_boat['time'] >= t1) & (data_boat['time'] <= t1_bis)]
docking2_boat = data_boat[(data_boat['time'] >= t2) & (data_boat['time'] <= t2_bis)]
L_boat_data = [docking1_boat, docking2_boat]
for data in L_boat_data:
    data.reset_index(drop=True, inplace=True)

docking1_stereo = data_stereo[(data_stereo['time'] >= t1) & (data_stereo['time'] <= t1_bis)]
docking2_stereo = data_stereo[(data_stereo['time'] >= t2) & (data_stereo['time'] <= t2_bis)]
L_stereo_data = [docking1_stereo, docking2_stereo]
for data in L_stereo_data:
    data.reset_index(drop=True, inplace=True)

print("data loaded \n docking 1 on time [",t1,",",t1_bis,"] \n docking 2 on time [",t2,",",t2_bis,"]")

# -----------------------------------
# uncertainty evaluation
# -----------------------------------

# stereo uncertainty
for i in range(len(L_stereo_data)):
    for j in range(len(L_stereo_data[i])):
        px_int = cdc.Interval(L_stereo_data[i]['x_px'].iloc[j]-889).inflate(px_error)
        focal_length_px_int = cdc.Interval(L_stereo_data[i]['focal_length_px'].iloc[j]).inflate(focal_error)
        disparity_int = cdc.Interval(L_stereo_data[i]['disparity'].iloc[j]).inflate(disparity_error)
        stereo_uncertainty = interval_uncertainty_propagation_stero(px_int, focal_length_px_int, disparity_int, baseline_int) + stereo_constant_uncertainty
        L_stereo_data[i].loc[j,"sensor_x_lb"] = stereo_uncertainty[0].lb()
        L_stereo_data[i].loc[j, "sensor_x_ub"] = stereo_uncertainty[0].ub()
        L_stereo_data[i].loc[j,"sensor_y_lb"] = stereo_uncertainty[1].lb()
        L_stereo_data[i].loc[j, "sensor_y_ub"] = stereo_uncertainty[1].ub()

#---------------------
# generate trajectories
#---------------------
L_boat_traj = [generate_traj(data) for data in L_boat_data]
L_stereo_traj = [generate_traj(data) for data in L_stereo_data]

L_stereo_traj_x_lb = [generate_bounding_traj(data,"sensor_x_lb") for data in L_stereo_data]
L_stereo_traj_x_ub = [generate_bounding_traj(data,"sensor_x_ub") for data in L_stereo_data]
L_stereo_traj_y_lb = [generate_bounding_traj(data,"sensor_y_lb") for data in L_stereo_data]
L_stereo_traj_y_ub = [generate_bounding_traj(data,"sensor_y_ub") for data in L_stereo_data]

#---------------------
# generate the estimation tubes
#---------------------
ctc_x_positif = cdc.CtcFunction(cdc.Function("x[2]", "x[0]"), cdc.Interval(0,cdc.oo))
L_boat_tube = [cdc.TubeVector(L_boat_traj[i],dt).inflate(ship_position_error) for i in range(2)]
for tube in L_boat_tube:
    ctc_x_positif.contract(tube)

L_stereo_tube_x = [cdc.Tube(L_stereo_traj_x_lb[i],dt) | L_stereo_traj_x_ub[i] for i in range(2)]
L_stereo_tube_y = [cdc.Tube(L_stereo_traj_y_lb[i],dt) | L_stereo_traj_y_ub[i] for i in range(2)]
L_stereo_tube = [cdc.TubeVector([L_stereo_tube_x[i],L_stereo_tube_y[i]]) for i in range(2)]

print("tubes generated")

if __name__ == '__main__':

    # Plot data from the boat
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Boat trajectory on the full dataset', fontsize=16)
    ax1.plot(data_boat['time'], data_boat['sensor_y'], 'bo', label="Boat py", markersize=4)
    ax1.plot(data_stereo['time'], data_stereo['sensor_y'], 'ro', label="Stereo py", markersize=2)
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Py')
    ax1.set_title("Y coordinates")
    ax1.grid(True)
    ax1.legend()

    # Plot data from the land platform
    ax2.plot(data_boat['time'], data_boat['sensor_x'], 'go', label="Boat px", markersize=4)
    ax2.plot(data_stereo['time'], data_stereo['sensor_x'], 'mo', label="Stereo px", markersize=2)
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Px')
    ax2.set_title('X coordinates')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()

    # plot a zoom on the docking data in a new figure
    plt.figure(figsize=(8, 4))

    plt.subplot(223)
    plt.plot(docking1_stereo['time'], docking1_stereo['sensor_y'], 'rv')
    plt.plot(docking1_stereo['time'], docking1_stereo['sensor_y_lb'], 'r--')
    plt.plot(docking1_stereo['time'], docking1_stereo['sensor_y_ub'], 'r--')
    plt.plot(docking1_boat['time'], docking1_boat['sensor_y'], 'b^')
    plt.xlabel('Time (s)')
    plt.ylabel('Py (m)')
    # plt.ylim([-100, 100])
    # plt.title("Docking 1")
    plt.grid(True)
    # plt.legend()

    plt.subplot(221)
    plt.plot(docking1_stereo['time'], docking1_stereo['sensor_x'], 'rv')
    plt.plot(docking1_stereo['time'], docking1_stereo['sensor_x_lb'], 'r--')
    plt.plot(docking1_stereo['time'], docking1_stereo['sensor_x_ub'], 'r--')
    plt.plot(docking1_boat['time'], docking1_boat['sensor_x'], 'b^')
    # plt.xlabel('Time')
    plt.ylabel('Px (m)')
    plt.title("Docking 1")
    # plt.ylim([-10,300])
    plt.grid(True)
    # plt.legend()

    plt.subplot(224)
    plt.plot(docking2_stereo['time'], docking2_stereo['sensor_y'], 'rv')
    plt.plot(docking2_stereo['time'], docking2_stereo['sensor_y_lb'], 'r--')
    plt.plot(docking2_stereo['time'], docking2_stereo['sensor_y_ub'], 'r--')
    plt.plot(docking2_boat['time'], docking2_boat['sensor_y'], 'b^')
    plt.xlabel('Time (s)')
    # plt.ylabel('Y values')
    # plt.title("Docking 2")
    plt.grid(True)
    # plt.ylim([-100, 100])
    # plt.legend()

    plt.subplot(222)
    plt.plot(docking2_stereo['time'], docking2_stereo['sensor_x'], 'rv', label="stereo")
    plt.plot(docking2_stereo['time'], docking2_stereo['sensor_x_lb'], 'r--',label="Stereo uncertainty")
    plt.plot(docking2_stereo['time'], docking2_stereo['sensor_x_ub'], 'r--')
    plt.plot(docking2_boat['time'], docking2_boat['sensor_x'], 'b^',label="ship")
    # plt.xlabel('Time')
    # plt.ylabel('X values')
    plt.title("Docking 2")
    # plt.ylim([-10, 300])
    plt.grid(True)
    plt.legend()

    plt.tight_layout()

    # save current figure
    plt.savefig('images/data.png',dpi=300)

    # plot the tubes
    # color_boat = "#A02400[#A0240077]"
    # color_stereo = "#0074A0[##0074A077]"
    # cdc.beginDrawing()
    # fig_1x = cdc.VIBesFigTube("docking 1 - px")
    # fig_1x.set_properties(0, 0, 600, 300)
    # fig_1x.add_tube(L_boat_tube[0][0], "boat",color_frgrnd=color_boat)
    # fig_1x.add_tube(L_stereo_tube_x[0], "stereo",color_frgrnd=color_stereo)
    # fig_1x.add_trajectory(L_boat_traj[0][0],"boat_traj",color=color_boat)
    # fig_1x.add_trajectory(L_stereo_traj[0][0],"stereo_traj",color=color_stereo)
    # fig_1x.show()
    #
    # fig_1y = cdc.VIBesFigTube("docking 1 - py")
    # fig_1y.set_properties(600, 0, 600, 300)
    # fig_1y.add_tube(L_boat_tube[0][1], "boat",color_frgrnd=color_boat)
    # fig_1y.add_tube(L_stereo_tube_y[0], "stereo",color_frgrnd=color_stereo)
    # fig_1y.add_trajectory(L_boat_traj[0][1],"boat_traj",color=color_boat)
    # fig_1y.add_trajectory(L_stereo_traj[0][1],"stereo_traj",color=color_stereo)
    # fig_1y.show()
    #
    # fig_2x = cdc.VIBesFigTube("docking 2 - px")
    # fig_2x.set_properties(0, 300, 600, 300)
    # fig_2x.add_tube(L_boat_tube[1][0], "boat",color_frgrnd=color_boat)
    # fig_2x.add_tube(L_stereo_tube_x[1], "stereo",color_frgrnd=color_stereo)
    # fig_2x.add_trajectory(L_boat_traj[1][0],"boat_traj",color=color_boat)
    # fig_2x.add_trajectory(L_stereo_traj[1][0],"stereo_traj",color=color_stereo)
    # fig_2x.show()
    #
    # fig_2y = cdc.VIBesFigTube("docking 2 - py")
    # fig_2y.set_properties(600, 300, 600, 300)
    # fig_2y.add_tube(L_boat_tube[1][1], "boat",color_frgrnd=color_boat)
    # fig_2y.add_tube(L_stereo_tube_y[1], "stereo",color_frgrnd=color_stereo)
    # fig_2y.add_trajectory(L_boat_traj[1][1],"boat_traj",color=color_boat)
    # fig_2y.add_trajectory(L_stereo_traj[1][1],"stereo_traj",color=color_stereo)
    # fig_2y.show()

    plt.show()
