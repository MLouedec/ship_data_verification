from tube_trajectory_generation import *

# -----------------------------------------
# value error
#------------------------------------------
docking1_boat_value_fault = L_boat_data[0].copy()

for i in range(len(docking1_boat_value_fault)):
    if docking1_boat_value_fault.loc[i,'time'] > t_fault1:
        docking1_boat_value_fault.loc[i,'sensor_x'] = docking1_boat_value_fault.loc[i,'sensor_x'] + c1*(docking1_boat_value_fault.loc[i,'time']-t_fault1)
        docking1_boat_value_fault.loc[i,'sensor_y'] = docking1_boat_value_fault.loc[i,'sensor_y'] + c2

docking1_boat_traj_value_fault = generate_traj(docking1_boat_value_fault)
docking1_boat_tube_value_fault =  cdc.TubeVector(docking1_boat_traj_value_fault,dt).inflate(ship_position_error)
ctc_x_positif.contract(docking1_boat_tube_value_fault)

# -----------------------------------------
# delay
# -----------------------------------------

docking2_boat_shifted = L_boat_data[1].copy()
closest_row = docking2_boat_shifted.iloc[(docking2_boat_shifted['time'] - t_fault_2).abs().argsort()[:1]].copy()
closest_row['time'] = t_fault_2-1
docking2_boat_shifted = pd.concat([docking2_boat_shifted, closest_row], ignore_index=True)
docking2_boat_shifted = docking2_boat_shifted.sort_values('time').reset_index(drop=True)
docking2_boat_shifted.loc[docking2_boat_shifted['time'] > t_fault_2,'time'] = docking2_boat_shifted['time'] + tau # delay of 40 sec
# discard line after time ti
ti = L_boat_data[1].iloc[-1]['time']
docking2_boat_shifted = docking2_boat_shifted[docking2_boat_shifted['time'] < ti]
last_row = docking2_boat_shifted.iloc[-1].copy()
last_row['time'] = ti
docking2_boat_shifted = pd.concat([docking2_boat_shifted, pd.DataFrame([last_row])], ignore_index=True)

docking2_boat_traj_shifted = generate_traj(docking2_boat_shifted)
docking2_boat_tube_shifted = cdc.TubeVector(docking2_boat_traj_shifted,dt).inflate(ship_position_error)
ctc_x_positif.contract(docking2_boat_tube_shifted)

print("tubes falsified")