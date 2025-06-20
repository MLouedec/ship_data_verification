import pandas as pd
import matplotlib.pyplot as plt
import codac as cdc
import numpy as np

pd.options.mode.chained_assignment = None

# ----------------------------------
# parameters
# ----------------------------------
baseline = 1.98 # m
baseline_int = cdc.Interval(baseline).inflate(0.01) # +-0,01m on the baseline

px_error = 5 # +- 5 horizontal pixel error on the position of the boat
focal_error = 60 # about 5% pixels error on the focal length
disparity_error = 5 # +- 5 pixel of disparity error

# for now one don't know how the boat is orientated and where is the antena in the boat
stereo_constant_uncertainty = cdc.IntervalVector([[-6,20],[-20,20]])

ship_position_error = 0.08 #m  ~ 0.01 m + 10m * sin(0,007 rad)

dt = 1 # sec discretisation step of the tubes

# value fault parameters f(t) = [c1*(t-t_fault_1) , c2]
t_fault1 = 380
c1 = 0.5
c2 = 15

# delay fault
t_fault_2 = 1600
tau = 60 # delay

# generate a 2d trajectories from the dataset
def generate_traj(dataset):
    x_values = {}
    y_values = {}
    for i in range(len(dataset)):
        x_values[dataset["time"].iloc[i]] = dataset["sensor_x"].iloc[i]
        y_values[dataset["time"].iloc[i]] = dataset["sensor_y"].iloc[i]
    traj = cdc.TrajectoryVector([cdc.Trajectory(x_values),cdc.Trajectory(y_values)])
    return traj

# generate a 1D trajectory from the dataset and a specified column name
def generate_bounding_traj(data,column):
    # 1d trajectory for the column
    values = {}
    for i in range(len(data)):
        if (pd.isna(data[column].iloc[i]) or np.isinf(data[column].iloc[i])):
            continue
        values[data["time"].iloc[i]] = data[column].iloc[i]
    traj = cdc.Trajectory(values)
    return traj

# compute an interval evaluation of sensor_x sensor_y (boat position estimated by the stereo cameras)
# px_int horizontal pixel position of the boat in the frame (0 is the center of the image)
# focal_lenght_px_int : focal lenght in pixel units
# disparity_int : disarity in pixel
# baseline_int : baseline in meter
def interval_uncertainty_propagation_stero(px_int,focal_length_px_int,disparity_int,baseline_int):
    sensor_x_int = focal_length_px_int*baseline_int/disparity_int # distance to the camera
    sensor_y_int = ((px_int)/disparity_int+cdc.Interval(0.5))*baseline_int
    return cdc.IntervalVector([sensor_x_int,sensor_y_int])

# compute the overlapping function between two tubes
# tube_a,tube_b : tubes to be compared
# tube_a can have a fault
# tube_b is trustworthy
def eval_overlap(tube_a,tube_b):
    intersection_tube = tube_a & tube_b

    L_t = np.linspace(tube_a.tdomain().lb(),tube_a.tdomain().ub(),10000)
    L_overlap = []
    for t in L_t:
        inter = intersection_tube(t)
        a_int = tube_a(t)
        overlap = inter.volume() / a_int.volume()
        L_overlap.append(overlap)
    return L_t, L_overlap

# evaluate the intervals of fault time with a threshold on the number of consecutive faults
# data : dataframe with the columns "time" and "overlap"
# time_alarm_threshold : threshold on the number of faults to trigger an alarm
def eval_fault_intervals(data,time_alarm_threshold):
    f = False
    ca = 0
    t_left = 0.
    res = []
    for i in range(len(data)):
        if f == False and data["overlap"].iloc[i] == 0: # new fault detected
            f = True
            ca = 1
            t_left = data["time"].iloc[i]
        elif f == True and data["overlap"].iloc[i] == 0: # still on the same fault
            ca += 1
        elif f == True and data["overlap"].iloc[i] > 0: # end of the fault
            f = False
            if ca > time_alarm_threshold:
                res.append(cdc.Interval([t_left,data["time"].iloc[i-1]]))
            ca = 0
        if i == len(data)-1 and f == True and ca >time_alarm_threshold:
                res.append(cdc.Interval([t_left,data["time"].iloc[i]]))
    return res

# def eval_delay(delay_init,tube_a,tube_b,e):
#     S = [(delay_init,tube_b)]
#     Res = []
#     ti = tube_a.tdomain().mid()
#     while len(S) > 0:
#         delay_i,tube_bi = S.pop()
#         cdc.ctc.delay.contract(delay_i, tube_bi, cdc.TubeVector(tube_a))
#         if delay_i.is_empty() or tube_a.is_empty() or tube_bi.is_empty():
#             continue
#         elif tube_bi(ti).volume() < e:
#             Res.append(delay_i)
#         else:
#             try:
#                 # ti = random.uniform(tube_bi.tdomain().lb(),tube_bi.tdomain().ub())
#                 Tube_bl,Tube_br = tube_bi.bisect(ti)
#                 S.append((cdc.Interval(delay_i),Tube_bl))
#                 S.append((cdc.Interval(delay_i),Tube_br))
#             except:
#                 # not bissectable,
#                 continue
#     return Res