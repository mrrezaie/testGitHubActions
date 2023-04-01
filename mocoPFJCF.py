import opensim as osim
import numpy as np
from scipy.signal import filtfilt, butter

# only one step
t0 = 0.245 # init time
t1 = 0.630 # end time # stride = 1.025 

# add extra things to the scaled model
model = osim.Model('out_scaled.osim')

# model.set_ForceSet(osim.ForceSet()) # remove all forces
model.set_MarkerSet(osim.MarkerSet()) # remove all markers
state = model.initSystem()

# # external load setup for right foot
# extLoad = osim.ExternalLoads()
# extLoad.setName('extLoad_r')
# extLoad.setDataFileName('exp_run.mot')
# ext1 = osim.ExternalForce()
# ext1.setName('right')
# ext1.set_applied_to_body('calcn_r')
# ext1.set_force_identifier('R_ground_force_v')
# ext1.set_point_identifier('R_ground_force_p')
# ext1.set_torque_identifier('R_ground_torque_')
# ext1.set_force_expressed_in_body('ground')
# ext1.set_point_expressed_in_body('ground')
# ext1.set_data_source_name('Unassigned')
# extLoad.cloneAndAppend(ext1)
# extLoad.printToXML('setup_extLoad_r.xml')

# add residual and reserve actuators
nfC = 0 # total number of free coordinates
for i in model.getCoordinateSet():
	path = i.getAbsolutePathString()
	if i.getMotionType()!=3 and i.get_locked()==False:
		nfC += 1
		CA = osim.CoordinateActuator()
		CA.setCoordinate(i)
		CA.setMinControl(-np.inf)
		CA.setMaxControl(+np.inf)
		if 'pelvis' in i.getName(): 
			CA.setName(i.getName()+'_residual')
			CA.setOptimalForce(200)
		else: 
			CA.setName(i.getName()+'_reserve')
			if 'lumbar' in i.getName():
				CA.setOptimalForce(100) # strong
			else:
				CA.setOptimalForce(1) # weak
		model.addForce(CA)

# add contact geometries (only right foot)
S1 = osim.ContactSphere(0.03, osim.Vec3([0.02,0,0]),
						model.getBodySet().get('calcn_r'), 'heel_r')
S2 = osim.ContactSphere(0.02, osim.Vec3([0.1,-0.001,-0.02]),
						model.getBodySet().get('calcn_r'), 'mid1_r')
S3 = osim.ContactSphere(0.02, osim.Vec3([0.09,-0.001,0.02]),
						model.getBodySet().get('calcn_r'), 'mid2_r')
S4 = osim.ContactSphere(0.02, osim.Vec3([0.18,-0.001,-0.03]),
						model.getBodySet().get('calcn_r'), 'fore1_r')
S5 = osim.ContactSphere(0.02, osim.Vec3([0.16,-0.001,0]),
						model.getBodySet().get('calcn_r'), 'fore2_r')
S6 = osim.ContactSphere(0.02, osim.Vec3([0.14,-0.001,0.03]),
						model.getBodySet().get('calcn_r'), 'fore3_r')
S7 = osim.ContactSphere(0.02, osim.Vec3([0.23,-0.001,-0.01]),
						model.getBodySet().get('calcn_r'), 'toe1_r')
S8 = osim.ContactSphere(0.02, osim.Vec3([0.21,-0.001,0.03]),
						model.getBodySet().get('calcn_r'), 'toe2_r')
floor = osim.ContactHalfSpace(osim.Vec3([0.5,0,-0.25]), osim.Vec3([0,0,-np.pi/2]),
							  model.getGround(), 'ground')
model.addContactGeometry(S1)
model.addContactGeometry(S2)
model.addContactGeometry(S3)
model.addContactGeometry(S4)
model.addContactGeometry(S5)
model.addContactGeometry(S6)
model.addContactGeometry(S7)
model.addContactGeometry(S8)
model.addContactGeometry(floor)

contacts = {'S1F': osim.SmoothSphereHalfSpaceForce('ground_heel_r',  S1, floor), 
			'S2F': osim.SmoothSphereHalfSpaceForce('ground_mid1_r',  S2, floor), 
			'S3F': osim.SmoothSphereHalfSpaceForce('ground_mid2_r',  S3, floor), 
			'S4F': osim.SmoothSphereHalfSpaceForce('ground_fore1_r', S4, floor), 
			'S5F': osim.SmoothSphereHalfSpaceForce('ground_fore2_r', S5, floor), 
			'S6F': osim.SmoothSphereHalfSpaceForce('ground_fore3_r', S6, floor), 
			'S7F': osim.SmoothSphereHalfSpaceForce('ground_toe1_r',  S7, floor), 
			'S8F': osim.SmoothSphereHalfSpaceForce('ground_toe2_r',  S8, floor)}

for i in contacts.keys():
	contacts[i].set_stiffness(3067776)
	contacts[i].set_dissipation(2)
	contacts[i].set_static_friction(0.8)
	contacts[i].set_dynamic_friction(0.8)
	contacts[i].set_hertz_smoothing(300)
	contacts[i].set_hunt_crossley_smoothing(50)
	contacts[i].set_viscous_friction(0.5)
	contacts[i].set_transition_velocity(0.2)
	contacts[i].set_constant_contact_force(1e-5)
	model.addForce(contacts[i])

# set static motion as default pose
static = osim.TimeSeriesTable('out_static.mot')
for i in model.getCoordinateSet():
	i.set_default_value(static.getDependentColumn(i.getAbsolutePathString()+'/value').getElt(0,0))

# read ik file 
fileIK = osim.TimeSeriesTable('out_ik.mot')
model.getSimbodyEngine().convertDegreesToRadians(fileIK)
time = np.array(fileIK.getIndependentColumn())
fs = round(1/(time[1]-time[0]))
f0 = list(time).index(t0) # find time range based on frame
f1 = list(time).index(t1)
time = time[f0:f1+1]
row = len(time)
q = {'time': time}
u = {'time': time}
# apply filter [may be unnecessary]
b,a = butter(4, 2*15/fs, btype='lowpass', output='ba')
for i in fileIK.getColumnLabels():
	temp = fileIK.getDependentColumn(i).to_numpy()[f0:f1+1]
	q[i] = filtfilt(b,a, temp, padlen=10)
	u[i] = filtfilt(b,a, np.gradient(temp, edge_order=2), padlen=10)

# collate state variables
s, nameS = list(), list()
s.append(q['time'])
nameS.append('time')

for i in model.getCoordinateSet():
	# set value
	nameS.append(i.getAbsolutePathString()+'/value')
	s.append(q[i.getName()])
	# set speed
	nameS.append(i.getAbsolutePathString()+'/speed')
	s.append(u[i.getName()])

# for i in model.getMuscles():
# 	# set activation
# 	nameS.append(i.getAbsolutePathString()+'/activation')
# 	s.append(np.ones(row)*0.01)
	# # set fiber_length
	# nameS.append(i.getAbsolutePathString()+'/fiber_length')
	# s.append(np.ones(row)*0.01)

# for i in model.getForceSet():
# 	# set activation
# 	nameS.append(i.getAbsolutePathString())
# 	s.append(np.ones(row)*0.01)

# write state file
col = len(nameS)
# nF = model.getForceSet().getSize()      # total number of controls
# nM = model.getMuscles().getSize()       # total number of muscles
# nC = model.getCoordinateSet().getSize() # total number of coordinates
# nS = nC + nF                            # total number of states
head =  f'state\nversion=1\nnRows={row}\nnColumns={col}\ninDegrees=yes\nendheader\n' + '\t'.join(nameS)
np.savetxt('out_state.sto', np.transpose(s), 
	fmt='%.6f', delimiter='\t', newline='\n', header=head, comments='')

model.finalizeConnections()
model.printToXML('out_scaled_upd.osim')

# %% Tracking
######################################################################

track = osim.MocoTrack()
track.setName('running_track')

# # coordinate tracking
# tableProc = osim.TableProcessor('out_ik.mot')
# tableProc.append(osim.TabOpLowPassFilter(15))
# track.setStatesReference(tableProc)
# track.set_allow_unused_references(True)
# track.set_states_global_tracking_weight(10)
# track.set_track_reference_position_derivatives(True)
# track.set_apply_tracked_states_to_guess(True)

# marker tracking
track.setMarkersReferenceFromTRC('exp_markers.trc', lowpassFilterFreq=15)
track.set_allow_unused_references(True)
track.set_markers_global_tracking_weight(10) # weight of MocoMarkerTrackingGoal
markerWeights = osim.MocoWeightSet()
markerWeights.cloneAndAppend(osim.MocoWeight('R.Shoulder', 2))
markerWeights.cloneAndAppend(osim.MocoWeight('L.Shoulder', 2))
markerWeights.cloneAndAppend(osim.MocoWeight('R.Clavicle', 2))
markerWeights.cloneAndAppend(osim.MocoWeight('L.Clavicle', 2))
markerWeights.cloneAndAppend(osim.MocoWeight('R.ASIS', 10))
markerWeights.cloneAndAppend(osim.MocoWeight('L.ASIS', 10))
markerWeights.cloneAndAppend(osim.MocoWeight('R.PSIS', 10))
markerWeights.cloneAndAppend(osim.MocoWeight('L.PSIS', 10))
markerWeights.cloneAndAppend(osim.MocoWeight('S2', 10))
markerWeights.cloneAndAppend(osim.MocoWeight('L.TH1', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.TH2', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.TH3', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.TH1', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.TH2', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.TH3', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.SH1', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.SH2', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.SH3', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.SH1', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.SH2', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.SH3', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.Knee', 3))
markerWeights.cloneAndAppend(osim.MocoWeight('R.Ankle', 3))
markerWeights.cloneAndAppend(osim.MocoWeight('L.Knee', 3))
markerWeights.cloneAndAppend(osim.MocoWeight('L.Ankle', 3))
markerWeights.cloneAndAppend(osim.MocoWeight('R.Heel', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.Toe', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('R.MT5', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.Heel', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.Toe', 4))
markerWeights.cloneAndAppend(osim.MocoWeight('L.MT5', 4))
track.set_markers_weight_set(markerWeights)

track.set_guess_file('out_state.sto')
# model = osim.Model('out_scaled_upd.osim')
# model.buildSystem()
# model.initializeState()
# model.getWorkingState()
modelProc = osim.ModelProcessor('out_scaled_upd.osim')
modelProc.append(osim.ModOpAddExternalLoads('setup_extLoad.xml'))
modelProc.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProc.append(osim.ModOpIgnoreTendonCompliance())
modelProc.append(osim.ModOpIgnorePassiveFiberForcesDGF())
modelProc.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5)) # why???
# modelProc.append(osim.ModOpAddReserves(1))
# modelProc.process().printToXML('out_scaled_upd.osim')

track.setModel(modelProc)
track.set_minimize_control_effort(True)
track.set_control_effort_weight(0.001) # default weight
track.set_initial_time(t0)
track.set_final_time(t1)
track.set_mesh_interval(0.08) # ???

study = track.initialize()
problem = study.updProblem()
model = modelProc.process()
model.initSystem()

########## Bounds
# set joint RoM as min and max bounds for each free coordinate
for i in model.getCoordinateSet():
	if i.getMotionType()!=3 and i.get_locked()==False:
		name = i.getAbsolutePathString()+'/value'
		problem.setStateInfo(name, [i.getRangeMin(), i.getRangeMax()])

########## Goals
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
# effort.setWeight(1)

# why???
forceSet = model.getForceSet()
for i in range(forceSet.getSize()):
    forcePath = forceSet.get(i).getAbsolutePathString()
    if 'pelvis' in forcePath:
        effort.setWeightForControl(forcePath, 10)

# contact tracking goal (weight = 1).
contact = osim.MocoContactTrackingGoal('contact_r', 1)
contact.setExternalLoadsFile('setup_extload.xml')
nameContacts = osim.StdVectorString()
nameContacts.append('/forceset/ground_heel_r');  nameContacts.append('/forceset/ground_mid1_r')
nameContacts.append('/forceset/ground_mid2_r');  nameContacts.append('/forceset/ground_fore1_r')
nameContacts.append('/forceset/ground_fore2_r'); nameContacts.append('/forceset/ground_fore3_r')
nameContacts.append('/forceset/ground_toe1_r');  nameContacts.append('/forceset/ground_toe2_r')
contact.addContactGroup(nameContacts, 'right')
contact.setProjection('plane')
contact.setProjectionVector(osim.Vec3(0, 0, 1))
problem.addGoal(contact)

########## Solver
# solver = study.initCasADiSolver()
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
# solver.set_num_mesh_intervals(50)
# solver.set_verbosity(2)
# solver.set_optim_solver("ipopt")
# solver.set_parameters_require_initsystem(False)
solver.set_optim_constraint_tolerance(1e-3)
solver.set_optim_convergence_tolerance(1e-3)
# solver.set_optim_max_iterations(1000)
# solver.set_parallel(2)

# set coordinates value and speed as default
initGuess = solver.createGuess()
fileState = osim.TimeSeriesTable('out_state.sto')
for i in model.getCoordinateSet():
	x = osim.Vector(fileState.getIndependentColumn())
	nx = initGuess.getTime()
	for j in ['/value', '/speed']:
		name = i.getAbsolutePathString()+j
		y = osim.Vector(fileState.getDependentColumn(name).to_numpy())
		vector = osim.interpolate(x,y,nx)
		initGuess.setState(name, vector)

# initGuess.write('out_tracking_init_guess.sto')
solver.setGuess(initGuess)

study.printToXML('out_tracking.moco')

trackingSolution = study.solve()
trackingSolution.write('out_tracking_solution.sto')
# study.visualize(trackingSolution)





# %% Prediction
######################################################################
# # calculate average running (stride) speed by pelvis_tx
# ik = osim.TimeSeriesTable('out_ik.mot')
# f0 = ik.getNearestRowIndexForTime(t0)
# f1 = ik.getNearestRowIndexForTime(t1)
# tx = ik.getDependentColumn('pelvis_tx').to_numpy()[f0:f1]
# # (tx[-1]-tx[0])/(t1-t0)  or np.gradient(tx).mean()*200
# speed = 2.7 # m/s

study = osim.MocoStudy()
study.setName('running_predict')

problem = study.updProblem()
modelProc = osim.ModelProcessor('out_scaled_upd.osim')
modelProc.append(osim.ModOpAddExternalLoads('setup_extLoad.xml'))
modelProc.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProc.append(osim.ModOpIgnoreTendonCompliance())
modelProc.append(osim.ModOpIgnorePassiveFiberForcesDGF())
modelProc.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
problem.setModelProcessor(modelProc)
problem.setTimeBounds(0.245, [0.6, 0.7])

########## Bounds
# set joint RoM as min and max bounds of each free coordinate
for i in model.getCoordinateSet():
	if i.getMotionType()!=3 and i.get_locked()==False:
		name = i.getAbsolutePathString()+'/value'
		problem.setStateInfo(name, [i.getRangeMin(), i.getRangeMax()])

########## Goals
speedGoal = osim.MocoAverageSpeedGoal('speed')
problem.addGoal(speedGoal)
speedGoal.set_desired_average_speed(2.7) # calculated from pelvis_tx global speed

# reaction goal
pfjcfGoal = osim.MocoJointReactionGoal('patellofemoral_compressive_force')
pfjcfGoal.setJointPath('/jointset/patellofemoral_r')
pfjcfGoal.setLoadsFrame('child')
pfjcfGoal.setExpressedInFramePath('/bodyset/patella_r') # child frame
pfjcfGoal.setReactionMeasures(['force-x']) # or All?
problem.addGoal(pfjcfGoal)

########## Solver
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(50) # ???
# solver.set_verbosity(2)
# solver.set_optim_solver('ipopt')
solver.set_optim_convergence_tolerance(1e-4)
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_max_iterations(1000)
solver.setGuess(trackingSolution) # initial guess

study.printToXML('out_prediction.moco')

# predictionSolution = study.solve()
# predictionSolution.write('out_prediction_solution.sto')
# study.visualize(predictionSolution)