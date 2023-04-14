import opensim as osim
import numpy as np
import os

# only one step
t0 = 0.245 # init time
t1 = 0.630 # end time # stride = 1.025 

# add extra things to the scaled model
model = osim.Model('out_scaled.osim')
model.initSystem()

# store right muscles
muscles = dict()
for i in model.getMuscles():
	if i.getName().endswith('_r'):
		i.set_ignore_activation_dynamics(False)
		i.set_ignore_tendon_compliance(False)
		i.set_max_contraction_velocity(25)
		muscles[i.getName()] = i.clone()

model.set_ForceSet(osim.ForceSet()) # remove all forces
model.set_MarkerSet(osim.MarkerSet()) # remove all markers

# remove left muscles
for ii in muscles.values():
		model.addForce(ii)


# add residual and reserve actuators
for i in model.getCoordinateSet():
	if i.getMotionType()!=3 and i.get_locked()==False:
		CA = osim.CoordinateActuator()
		CA.setCoordinate(i)
		CA.setMinControl(-np.inf)
		CA.setMaxControl(+np.inf)
		if 'pelvis' in i.getName(): 
			CA.setName(i.getName()+'_residual')
			CA.setOptimalForce(200)
		else: 
			CA.setName(i.getName()+'_reserve')
			if 'lumbar' in i.getName() or i.getName().endswith('_l'):
				CA.setOptimalForce(200) # strong
			else:
				CA.setOptimalForce(1) # weak
		model.addForce(CA)


# add contact geometries and forces (only right foot)
contacts = {
			'S1': osim.ContactSphere(0.03, osim.Vec3([0.02,0,0]),
						model.getBodySet().get('calcn_r'), 'heel_r'),
			'S2': osim.ContactSphere(0.02, osim.Vec3([0.1,-0.001,-0.02]),
						model.getBodySet().get('calcn_r'), 'mid1_r'),
			'S3': osim.ContactSphere(0.02, osim.Vec3([0.09,-0.001,0.02]),
						model.getBodySet().get('calcn_r'), 'mid2_r'),
			'S4': osim.ContactSphere(0.02, osim.Vec3([0.18,-0.001,-0.03]),
						model.getBodySet().get('calcn_r'), 'fore1_r'),
			'S5': osim.ContactSphere(0.02, osim.Vec3([0.16,-0.001,0]),
						model.getBodySet().get('calcn_r'), 'fore2_r'),
			'S6': osim.ContactSphere(0.02, osim.Vec3([0.14,-0.001,0.03]),
						model.getBodySet().get('calcn_r'), 'fore3_r'),
			'S7': osim.ContactSphere(0.02, osim.Vec3([0.23,-0.001,-0.01]),
						model.getBodySet().get('calcn_r'), 'toe1_r'),
			'S8': osim.ContactSphere(0.02, osim.Vec3([0.21,-0.001,0.03]),
						model.getBodySet().get('calcn_r'), 'toe2_r'),
			'floor': osim.ContactHalfSpace(osim.Vec3([0.5,0,-0.25]), osim.Vec3([0,0,-np.pi/2]),
							  model.getGround(), 'ground')}

for i in contacts.keys():
	model.addContactGeometry(contacts[i])

contactForces = {
			'S1F': osim.SmoothSphereHalfSpaceForce('ground_heel_r',  contacts['S1'], contacts['floor']), 
			'S2F': osim.SmoothSphereHalfSpaceForce('ground_mid1_r',  contacts['S2'], contacts['floor']), 
			'S3F': osim.SmoothSphereHalfSpaceForce('ground_mid2_r',  contacts['S3'], contacts['floor']), 
			'S4F': osim.SmoothSphereHalfSpaceForce('ground_fore1_r', contacts['S4'], contacts['floor']), 
			'S5F': osim.SmoothSphereHalfSpaceForce('ground_fore2_r', contacts['S5'], contacts['floor']), 
			'S6F': osim.SmoothSphereHalfSpaceForce('ground_fore3_r', contacts['S6'], contacts['floor']), 
			'S7F': osim.SmoothSphereHalfSpaceForce('ground_toe1_r',  contacts['S7'], contacts['floor']), 
			'S8F': osim.SmoothSphereHalfSpaceForce('ground_toe2_r',  contacts['S8'], contacts['floor'])}

for i in contactForces.keys():
	contactForces[i].set_stiffness(3067776)
	contactForces[i].set_dissipation(2)
	contactForces[i].set_static_friction(0.8)
	contactForces[i].set_dynamic_friction(0.8)
	contactForces[i].set_hertz_smoothing(300)
	contactForces[i].set_hunt_crossley_smoothing(50)
	contactForces[i].set_viscous_friction(0.5)
	contactForces[i].set_transition_velocity(0.2)
	contactForces[i].set_constant_contact_force(1e-5)
	model.addForce(contactForces[i])


# set static motion as default pose
static = osim.TimeSeriesTable('out_static.mot')
for i in model.getCoordinateSet():
	i.set_default_value(static.getDependentColumn(i.getAbsolutePathString()+'/value').getElt(0,0))


model.finalizeConnections()
model.initSystem()	
model.printToXML('out_scaled_upd.osim')


# read ik file and convert it to a state file (speeds are included)
fileIK = osim.TimeSeriesTable('out_ik.mot')
time = fileIK.getIndependentColumn()
dt = time[1]-time[0] # time interval (==0.005)
fileIK = osim.TableProcessor(fileIK)
fileIK.append(osim.TabOpLowPassFilter(15))
fileIK.append(osim.TabOpConvertDegreesToRadians())
fileIK.append(osim.TabOpUseAbsoluteStateNames())
tableIK = fileIK.process(model)
for i in tableIK.getColumnLabels():
	temp = tableIK.getDependentColumn(i).to_numpy()
	vector = osim.Vector(np.gradient(temp, edge_order=2)/dt) # speed
	tableIK.appendColumn(i[:-5]+'speed', vector)
tableIK.trim(t0, t1) # tableIK.trim(time[0], time[-1]) # trim to exclude padding
osim.STOFileAdapter.write(tableIK, 'out_state.sto')

# %% Tracking
######################################################################

import opensim as osim
import numpy as np
import os

# osim.ModelVisualizer.addDirToGeometrySearchPaths('.\\Geometry')
osim.ModelVisualizer.addDirToGeometrySearchPaths('C:\\OpenSim 4.3\\Geometry')

# only one step
t0 = 0.245 # init time
t1 = 0.630 # end time # stride = 1.025 

track = osim.MocoTrack()
track.setName('running_track')

# # coordinate tracking
# track.setStatesReference(osim.TableProcessor('out_state.sto'))
# track.set_allow_unused_references(True)
# track.set_states_global_tracking_weight(10)
# track.set_track_reference_position_derivatives(True)
# track.set_apply_tracked_states_to_guess(True)

# marker tracking
tableProc = osim.TableProcessor(osim.TimeSeriesTableVec3('exp_markers.trc').flatten(['x','y','z']))
tableProc.append(osim.TabOpLowPassFilter(15))
track.setMarkersReference(tableProc)
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

model = osim.Model('out_scaled_upd.osim')
modelProc = osim.ModelProcessor(model)
modelProc.append(osim.ModOpAddExternalLoads('setup_extLoad.xml'))
modelProc.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProc.append(osim.ModOpIgnoreActivationDynamics())
modelProc.append(osim.ModOpIgnoreTendonCompliance())
# modelProc.append(osim.ModOpUseImplicitTendonComplianceDynamicsDGF()) # tendon is already rigid
modelProc.append(osim.ModOpIgnorePassiveFiberForcesDGF())
modelProc.append(osim.ModOpScaleMaxIsometricForce(2)) # FUN???
modelProc.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
locked = osim.StdVectorString()
for i in model.getCoordinateSet():
	if i.get_locked():
		locked.append(i.getJoint().getName())
modelProc.append(osim.ModOpReplaceJointsWithWelds(locked))
# modelProc.append(osim.ModOpAddReserves(1)) # already included
# modelProc.process().printToXML('out_scaled_upd.osim')

track.setModel(modelProc)
track.set_minimize_control_effort(True)
# track.set_control_effort_weight(0.001) # default weight
track.set_initial_time(t0)
track.set_final_time(t1)
track.set_mesh_interval(0.02)

study = track.initialize()
problem = study.updProblem()
# model = modelProc.process()
# model.initSystem()

########## Bounds
# set joint RoM as min and max bounds for each free coordinate
for i in model.getCoordinateSet():
	if i.getMotionType()!=3 and i.get_locked()==False:
		name = i.getAbsolutePathString()+'/value'
		problem.setStateInfo(name, [i.getRangeMin(), i.getRangeMax()])

# problem.setStateInfoPattern('/forceset/.*/activation', [0, 1])
# problem.setStateInfoPattern('/forceset/.*/normalized_tendon_force', [0, 1.8], [], []);

# set activation bounds only for muscles (not all forces)
# for i in model.getMuscles():
# 	name = i.getAbsolutePathString()
# 	problem.setStateInfoPattern(name+'/activation', [0.01, 1])
	# problem.setStateInfoPattern(name+'/fiber_length', [0, 1])
	# problem.setStateInfoPattern(name+'/normalized_tendon_force', [0, 1])

########## Goals
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
effort.setWeight(0.01)

# why???
for i in model.getForceSet():
    forcePath = i.getAbsolutePathString()
    if 'pelvis' in forcePath:
        effort.setWeightForControl(forcePath, 10)

# contact tracking goal (weight = 1).
contact = osim.MocoContactTrackingGoal('contact_r', 1)
contact.setExternalLoadsFile('setup_extload.xml')
nameContacts = osim.StdVectorString()
for i in ['heel_r', 'mid1_r', 'mid2_r', 'fore1_r', 'fore2_r', 'fore3_r', 'toe1_r', 'toe2_r']:
	nameContacts.append('/forceset/ground_'+i)
contact.addContactGroup(nameContacts, 'right')
contact.setProjection('plane')
contact.setProjectionVector(osim.Vec3(0, 0, 1))
# problem.addGoal(contact) # Not included yet

########## Solver
# solver = study.initCasADiSolver()
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
# solver.set_num_mesh_intervals(50)
# solver.set_verbosity(2)
# solver.set_optim_solver("ipopt")
# solver.set_parameters_require_initsystem(False)
solver.set_optim_constraint_tolerance(1e-2)
solver.set_optim_convergence_tolerance(1e-2)
# solver.set_optim_max_iterations(1000)
# solver.set_parallel(2)

# set coordinates value and speed as initial guesses
initGuess = solver.createGuess('bounds')
fileState = osim.TimeSeriesTable('out_state.sto')
x = osim.Vector(fileState.getIndependentColumn())
nx = initGuess.getTime()
for i in model.getCoordinateSet():
	if i.get_locked()==False:
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
study.visualize(trackingSolution)

os.remove('running_track_tracked_markers.sto')







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
# solver.set_num_mesh_intervals(50) # ???
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
