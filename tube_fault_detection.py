from tube_trajectory_falsification import *

# no falsification
tube_b01 = cdc.TubeVector(L_stereo_tube[0]) # stereo
tube_a01 = cdc.TubeVector(L_boat_tube[0]) # boat

tube_b02 = cdc.TubeVector(L_stereo_tube[1])
tube_a02 = cdc.TubeVector(L_boat_tube[1])

# value false
tube_b1 = cdc.TubeVector(L_stereo_tube[0])
tube_a1 = cdc.TubeVector(docking1_boat_tube_value_fault)

# time false
tube_b2 = cdc.TubeVector(L_stereo_tube[1])
tube_a2 = cdc.TubeVector(docking2_boat_tube_shifted)

# time truncate
t_domain01 = tube_a01.tdomain() & tube_b01.tdomain()
t_domain02 = tube_a02.tdomain() & tube_b02.tdomain()
t_domain1 = tube_a1.tdomain() & tube_b1.tdomain()
t_domain2 = tube_a2.tdomain() & tube_b2.tdomain()

tube_a01.truncate_tdomain(t_domain01)
tube_b01.truncate_tdomain(t_domain01)
tube_a01.sample(tube_b01)

tube_a02.truncate_tdomain(t_domain02)
tube_b02.truncate_tdomain(t_domain02)
tube_a02.sample(tube_b02)

tube_a1.truncate_tdomain(t_domain1)
tube_b1.truncate_tdomain(t_domain1)
tube_a1.sample(tube_b1)

tube_a2.truncate_tdomain(t_domain2)
tube_b2.truncate_tdomain(t_domain2)
tube_a2.sample(tube_b2)

# ------------------------------------------
# overlapping function
# ------------------------------------------

L_t01, L_overlap01 = eval_overlap(tube_a01, tube_b01)
L_t02, L_overlap02 = eval_overlap(tube_a02, tube_b02)
L_t1, L_overlap1 = eval_overlap(tube_a1, tube_b1)
L_t2, L_overlap2 = eval_overlap(tube_a2, tube_b2)

data_t1 = pd.DataFrame({"time":L_t1,"overlap":L_overlap1})
data_t2 = pd.DataFrame({"time":L_t2,"overlap":L_overlap2})

# only keep the line where overlap is 0
faults_t1 = eval_fault_intervals(data_t1,5)
faults_t2 = eval_fault_intervals(data_t2,5)

fault_t1 = faults_t1[0]
fault_t2 = faults_t2[0]
fault_t2_bis = faults_t2[2]
fault_t2_ter = faults_t2[1]

print("Fault detected at t in ", faults_t1," for docking 1")
print("Fault detected at t in ",faults_t2," for docking 2")

# ------------------------------------------
# evaluate value fault, a = b + r with the residual r
# ------------------------------------------

# residual
tube_r01 = tube_b01-tube_a01
tube_r02 = tube_b02-tube_a02

tube_r1 = tube_b1-tube_a1
tube_r2 = tube_b2-tube_a2

tube_r1_fault = cdc.TubeVector(tube_r1)
tube_r1_fault.truncate_tdomain(fault_t1)
fault_r1_eval = tube_r1_fault.codomain()

tube_r2_fault = cdc.TubeVector(tube_r2)
tube_r2_fault.truncate_tdomain(fault_t2)
fault_r2_eval = tube_r2_fault.codomain()

print("codomain of r1 during fault",fault_r1_eval)
print("codomain of r2 during fault",fault_r2_eval)

# ------------------------------------------
# eval the delay fault
# ------------------------------------------

tube_a1_fault = cdc.TubeVector(tube_a1)
tube_a1_fault.truncate_tdomain(fault_t1) # tube a1 when fault is detected
tube_b1_fault = cdc.TubeVector(tube_b1)
tube_b1_fault.truncate_tdomain(fault_t1)

delay_max = fault_t1.diam()*0.9
print("delay max for fault 1",delay_max)
delay_1 = cdc.Interval(0,delay_max)

cdc.ctc.delay.contract(delay_1, tube_b1_fault, tube_a1_fault)
print("delay evaluated for fault 1",delay_1) # it is empty so no delay possible

# fault_t2 = fault_t2 + cdc.Interval(0,50) # peut être nécessaire d'ajouter du temps
fault_t2 = cdc.Interval(fault_t2.lb(),fault_t2_ter.ub()-1) # the -1 id added empirically to avoid using the end of the tube which apparently makes contraction less effective
delay_max = fault_t2.diam()*0.9
print("delay max for fault 2",delay_max)

tube_a2_fault = cdc.TubeVector(tube_a2)
tube_a2_fault.truncate_tdomain(fault_t2) # tube a2 when fault is detected
tube_b2_fault = cdc.TubeVector(tube_b2)
tube_b2_fault.truncate_tdomain(fault_t2)
delay_2 = cdc.Interval(0,delay_max)

cdc.ctc.delay.contract(delay_2, tube_b2_fault, tube_a2_fault)
print("delay evaluated for fault 2",delay_2) # it is empty so no delay possible

tube_a2_fault_bis = cdc.TubeVector(tube_a2)
tube_a2_fault_bis.truncate_tdomain(fault_t2_bis) # tube a2 when fault is detected
tube_b2_fault_bis = cdc.TubeVector(tube_b2)
tube_b2_fault_bis.truncate_tdomain(fault_t2_bis)
delay_2_bis = cdc.Interval(0,delay_max)

cdc.ctc.delay.contract(delay_2_bis, tube_b2_fault_bis, tube_a2_fault_bis)
print("delay evaluated for fault 2 bis",delay_2_bis) # it is empty so no delay possible

# plot the overlaps functions on the same figure
fig1 = plt.figure(1, figsize=(8, 3))

ax1, ax2 = fig1.add_subplot(121), fig1.add_subplot(122)
lwidth = 2
ax1.plot(L_t01, L_overlap01, color="k", linestyle="--", linewidth=lwidth)
ax1.plot(L_t1, L_overlap1, color="r", linewidth=lwidth)
ax1.axvline(x=t_fault1, color='g', linestyle=':',linewidth=lwidth)
ax2.plot(L_t02, L_overlap02, label="no fault", color="k", linestyle="--", linewidth=lwidth)
ax2.plot(L_t2, L_overlap2, label="with fault", color="r", linewidth=lwidth)
ax2.axvline(x=t_fault_2, color='g', linestyle=':', label='Fault time',linewidth=lwidth)
ax1.set_title("docking 1", fontsize=14)
ax2.set_title("docking 2", fontsize=14)
ax1.set_xlabel("time", fontsize=12)
ax1.set_ylabel("overlap", fontsize=12)
ax2.set_xlabel("time", fontsize=12)
# ax2.set_ylabel("overlap", fontsize=12)
# ax1.legend(fontsize=12, loc='upper right')
ax2.legend(fontsize=12, loc='upper right')
ax1.tick_params(axis='both', labelsize=10)
ax2.tick_params(axis='both', labelsize=10)

plt.subplots_adjust(bottom=0.2)

fig1.savefig("images/overlapping_function.png", dpi=300)

# plot the tubes
color_a = "#000000[#fab60077]"
color_b = "#2a5675[#2a567590]"
color_c = "#000000[#80808090]"

cdc.beginDrawing()

wi,he = 300,150
ver = 50
hor = 700

dk1pxn = cdc.VIBesFigTube("docking 1 - px - normal")
dk1pxn.set_properties(0, 8*ver, wi, he)
dk1pxn.add_tube(tube_a01[0], "a1",color_frgrnd=color_a)
dk1pxn.add_tube(tube_b01[0], "b1",color_frgrnd=color_b)
dk1pxn.show()

dk1pyn = cdc.VIBesFigTube("docking 1 - py - normal")
dk1pyn.set_properties(0,7*ver, wi, he)
dk1pyn.add_tube(tube_a01[1], "a1",color_frgrnd=color_a)
dk1pyn.add_tube(tube_b01[1], "b1",color_frgrnd=color_b)
dk1pyn.show()

dk2pxn = cdc.VIBesFigTube("docking 2 - px - normal")
dk2pxn.set_properties(0,6*ver, wi, he)
dk2pxn.add_tube(tube_a02[0], "a2",color_frgrnd=color_a)
dk2pxn.add_tube(tube_b02[0], "b2",color_frgrnd=color_b)
dk2pxn.show()

dk2pyn = cdc.VIBesFigTube("docking 2 - py - normal")
dk2pyn.set_properties(0,5*ver, wi, he)
dk2pyn.add_tube(tube_a02[1], "a2",color_frgrnd=color_a)
dk2pyn.add_tube(tube_b02[1], "b2",color_frgrnd=color_b)
dk2pyn.show()

dk1rxn = cdc.VIBesFigTube("r 1 - px - normal")
dk1rxn.set_properties(0,4*ver, wi, he)
dk1rxn.add_tube(tube_r01[0], "r1x",color_frgrnd=color_c)
dk1rxn.show()
#
dk1ryn = cdc.VIBesFigTube("r 1 - py - normal")
dk1ryn.set_properties(0,3*ver, wi, he)
dk1ryn.add_tube(tube_r01[1], "r1y",color_frgrnd=color_c)
dk1ryn.show()

dk2rxn = cdc.VIBesFigTube("r 2 - px - normal")
dk2rxn.set_properties(0,2*ver, wi, he)
dk2rxn.add_tube(tube_r02[0],"r2x",color_frgrnd=color_c)
dk2rxn.show()

dk2ryn = cdc.VIBesFigTube("r 2 - py - normal")
dk2ryn.set_properties(0,1*ver, wi, he)
dk2ryn.add_tube(tube_r02[1],"r2y",color_frgrnd=color_c)
dk2ryn.show()

dk1pxf = cdc.VIBesFigTube("docking 1 - px - value false")
dk1pxf.set_properties(hor,8*ver, wi, he)
dk1pxf.add_tube(tube_a1[0], "a1",color_frgrnd=color_a)
dk1pxf.add_tube(tube_b1[0], "b1",color_frgrnd=color_b)
dk1pxf.show()

dk1pyf = cdc.VIBesFigTube("docking 1 - py - value false")
dk1pyf.set_properties(hor, 7*ver, wi, he)
dk1pyf.add_tube(tube_a1[1], "a1",color_frgrnd=color_a)
dk1pyf.add_tube(tube_b1[1], "b1",color_frgrnd=color_b)
dk1pyf.show()

dk2pxf = cdc.VIBesFigTube("docking 2 - px - time false")
dk2pxf.set_properties(hor, 6*ver, wi, he)
dk2pxf.add_tube(tube_a2[0], "a2",color_frgrnd=color_a)
dk2pxf.add_tube(tube_b2[0], "b2",color_frgrnd=color_b)
dk2pxf.show()

dk2pyf = cdc.VIBesFigTube("docking 2 - py - time false")
dk2pyf.set_properties(hor, 5*ver, wi, he)
dk2pyf.add_tube(tube_a2[1], "a2",color_frgrnd=color_a)
dk2pyf.add_tube(tube_b2[1], "b2",color_frgrnd=color_b)
dk2pyf.show()

dk1rxf = cdc.VIBesFigTube("r 1 - px - value false")
dk1rxf.set_properties(hor, 4*ver, wi, he)
dk1rxf.add_tube(tube_r1[0], "r1x",color_frgrnd=color_c)
dk1rxf.show()
#
dk1ryf = cdc.VIBesFigTube("r 1 - py - value false")
dk1ryf.set_properties(hor, 3*ver, wi, he)
dk1ryf.add_tube(tube_r1[1], "r1y",color_frgrnd=color_c)
dk1ryf.show()

dk2rxf = cdc.VIBesFigTube("r 2 - px - time false")
dk2rxf.set_properties(hor, 2*ver, wi, he)
dk2rxf.add_tube(tube_r2[0],"r2x",color_frgrnd=color_c)
dk2rxf.show()

dk2ryf = cdc.VIBesFigTube("r 2 - py - time false")
dk2ryf.set_properties(hor, 1*ver, wi, he)
dk2ryf.add_tube(tube_r2[1],"r2y",color_frgrnd=color_c)
dk2ryf.show()
#
# fig_2x_zoom = cdc.VIBesFigTube("zoom error 2 - x")
# fig_2x_zoom.set_properties(0, 300, 600, 300)
# fig_2x_zoom.add_tube(tube_a2_fault[0], "a2",color_frgrnd=color_a)
# fig_2x_zoom.add_tube(tube_b2_fault[0], "b2",color_frgrnd=color_b)
# fig_2x_zoom.show()

cdc.endDrawing()
plt.show()

print("done")

