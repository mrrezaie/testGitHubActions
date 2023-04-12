import opensim as osim
import numpy as np
import os

# %% Define functions

def kinematicsToStates(kinematicsFileName = None, osimModelFileName = None,
                       outputFileName = 'coordinates.sto',
                       inDegrees = True, outDegrees = False):
    
    """
    
    Convenience function for converting IK results to a states storage.
    
    Input:    kinematicsFileName - file containing kinematic data. Header should only be coordinates name, rather than path to state
              osimModelFileName - opensim model filename that corresponds to kinematic data
              outputFileName - optional filename to output to (defaults to coordinates.sto)
              inDegrees - set to true if kinematics file is in degrees (defaults to True)
              outDegrees - set to true if desired output is in degrees (defaults to False)
    
    """
    
    #Check inputs
    if kinematicsFileName is None:
        raise ValueError('Filename for kinematics is required')
    if osimModelFileName is None:
        raise ValueError('OpenSim model filename is required')
    
    #Load in the kinematic data
    kinematicsStorage = osim.Storage(kinematicsFileName)
    
    #Create a copy of the kinematics data to alter the column labels in
    statesStorage = osim.Storage(kinematicsFileName)
    
    #Resample the data points linearly to avoid any later issues with matching
    #time points. Use a time stamp for 250 Hz
    kinematicsStorage.resampleLinear(1/250)
    statesStorage.resampleLinear(1/250)
    
    #Get the column headers for the storage file
    angleNames = kinematicsStorage.getColumnLabels()
    
    #Get the corresponding full paths from the model to rename the
    #angles in the kinematics file
    kinematicModel = osim.Model(osimModelFileName)
    for ii in range(0,angleNames.getSize()):
        currAngle = angleNames.get(ii)
        if currAngle != 'time':
            #Get full path to coordinate
            fullPath = kinematicModel.updCoordinateSet().get(currAngle).getAbsolutePathString()+'/value'
            #Set angle name appropriately using full path
            angleNames.set(ii,fullPath)
    
    #Set the states storage object to have the updated column labels
    statesStorage.setColumnLabels(angleNames)
    
    #Appropriately set output in degrees or radians
    if inDegrees and not outDegrees:
        #Convert degrees values to radians for consistency with the current
        #file label (defaults back to inDegrees=no). Radians seem to work
        #better with the Moco process as well.
        kinematicModel.initSystem()
        kinematicModel.getSimbodyEngine().convertDegreesToRadians(statesStorage)
    elif inDegrees and outDegrees:
        #Change the storage label back to specifying indegrees=yes
        statesStorage.setInDegrees(True)
    elif not inDegrees and outDegrees:
        #Convert radians to degrees
        kinematicModel.initSystem()
        kinematicModel.getSimbodyEngine().convertRadiansToDegrees(statesStorage)
        #Reset labeling for degrees
        statesStorage.setInDegrees(True)
    
    #Write the states storage object to file
    statesStorage.printToXML(outputFileName)

# %%

#Add OpenSim geometry path (weird issues with this on new laptop)
osim.ModelVisualizer.addDirToGeometrySearchPaths('C:\\OpenSim 4.3\\Geometry')

# only one step
t0 = 0.245 # init time
t1 = 0.630 # end time # stride = 1.025 

# Convert IK to states coordinates file
# Different approach to below
kinematicsToStates(kinematicsFileName = 'out_ik.mot', osimModelFileName = 'out_scaled.osim')

# %% Set-up baseline marker tracking simulation
#    Added by Aaron

"""

The below code creates and runs a marker tracking problem from the experimental
data and scaled model. The experimental external loads (GRFs) are included in this
problem via appending to the model. The optimal control problem is therefore made
up of the marker tracking errors and minimising the control signals. Lowly weighted
(via low optimal force) reserve actuators are appended to the model.

"""

#Construct a ModelProcessor and add it to the tool
modelProcessor = osim.ModelProcessor('out_scaled.osim')

#Add external loads to model
modelProcessor.append(osim.ModOpAddExternalLoads('setup_extload.xml'))

#Ignore tendon compliance
modelProcessor.append(osim.ModOpIgnoreTendonCompliance())

#Set model to use implicit mode
modelProcessor.append(osim.ModOpTendonComplianceDynamicsModeDGF('implicit'))

#Set model muscles to convert to the DeGrooteFregly2016 model which can be
#used with direct collocation in Moco
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())

#Ignore passive fiber forces in muscles
modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())

#Rescale active fiber force curve in muscles. This does seem to improve
#performance for gait motions
modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

#rescale isometric forces just for fun
modelProcessor.append(osim.ModOpScaleMaxIsometricForce(2))

#Add some reserve actuators to help problem
modelProcessor.append(osim.ModOpAddReserves(1))

#Process model for further edits
osimModel = modelProcessor.process()

#Update muscle contraction velocity
for muscleInd in range(osimModel.getMuscles().getSize()):
    osimModel.getMuscles().get(muscleInd).set_max_contraction_velocity(25)
    
#Finalise model
osimModel.finalizeConnections()

#Create a tracking tool
track = osim.MocoTrack()
track.setName('markerTracking')

#Set the model in tracking tool
track.setModel(osim.ModelProcessor(osimModel))

#Set marker data in tracking problem
#Note the need to flatten to a table based on info here:
#https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=1815&t=14612&p=43550&start=0&view=
markerTable = osim.TimeSeriesTableVec3('exp_markers.trc',)
tableProcessor = osim.TableProcessor(markerTable.flatten())
track.setMarkersReference(tableProcessor)

#Ensure that any unused data in trc file can be handled
track.set_allow_unused_references(True)

#Set the global markers tracking weight
track.set_markers_global_tracking_weight(10)

#Set marker weights
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

#Set times in tracking tool
track.set_initial_time(t0)
track.set_final_time(t1)
# track.set_mesh_interval(0.02)  ### TODO: could solve on a coarser grid to begin with??? Time interval set with solver mesh intervals later...

#Convert to Moco Study to update problem
study = track.initialize()
problem = study.updProblem()

#Update the weight on the default control effort goal
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
effort.setWeight(0.01)

#### TODO: there is an opportunity to set individualised control weights here too...

#Define the solver and set its options
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.set_optim_constraint_tolerance(0.01) #default = 0.01
solver.set_optim_convergence_tolerance(0.01) #default = 0.01
solver.set_num_mesh_intervals(10) ### this is quite coarse to begin with
solver.resetProblem(problem)

########## Bounds --- needs fixing, crashed my Python for some reason?
# set joint RoM as min and max bounds for each free coordinate
# model = modelProcessor.process()
# for i in model.getCoordinateSet():
# 	if i.getMotionType()!=3 and i.get_locked()==False:
# 		name = i.getAbsolutePathString()+'/value'
# 		problem.setStateInfo(name, [i.getRangeMin(), i.getRangeMax()])

#Set coordinates value and speed from IK
initGuess = solver.createGuess()
fileState = osim.TimeSeriesTable('out_state.sto')
for ii in osimModel.getCoordinateSet():
	x = osim.Vector(fileState.getIndependentColumn())
	nx = initGuess.getTime()
	for j in ['/value', '/speed']:
		name = ii.getAbsolutePathString()+j
		y = osim.Vector(fileState.getDependentColumn(name).to_numpy())
		vector = osim.interpolate(x,y,nx)
		initGuess.setState(name, vector)
        
#Set guess
solver.setGuess(initGuess)

#Solve!
solution = study.solve()

#Write to file
solution.write('initialMarkerTracking_solution.sto')

#Remove the tracked markers file
os.remove('_tracked_markers.sto')

"""

The marker tracking problem took a decent amount of time and iterations to solve,
even at a fairly coarse grid density (i.e. 21 nodes) — 3 hrs 47 mins with 1119
iterations. This problem will be inherently slow given the model is full body and
contains 80 muscles — however, any problem that takes > 1000 iterations isn't that
well conditioned. This could probably be improved by setting some bounds (overall
and/or initial coordinate values) or even by setting an initial guess from an IK
solution.

Using an initial guess from IK gets the solver in a better position earlier on and
it doesn't look like it will take as many iterations to solve.
    Had to stop at around 550 iterations, but had already reached an objective value
    of 0.3 at this stage and looking like low constraint violations etc. Noting that
    the original solutionsolved at 0.26 in 1119 iterations

"""

# %% Set-up baseline coordinate tracking simulation
#    Added by Aaron

"""

The below code creates and runs a coordinate tracking problem from the experimental
IK data and scaled model. The experimental external loads (GRFs) are included in this
problem via appending to the model. The optimal control problem is therefore made
up of the coordinate tracking errors and minimising the control signals. Lowly
weighted (via low optimal force) reserve actuators are appended to the model.

"""

#Construct a ModelProcessor and add it to the tool
modelProcessor = osim.ModelProcessor('out_scaled.osim')

#Add external loads to model
modelProcessor.append(osim.ModOpAddExternalLoads('setup_extload.xml'))

#Ignore tendon compliance
modelProcessor.append(osim.ModOpIgnoreTendonCompliance())

#Set model to use implicit mode
modelProcessor.append(osim.ModOpTendonComplianceDynamicsModeDGF('implicit'))

#Set model muscles to convert to the DeGrooteFregly2016 model which can be
#used with direct collocation in Moco
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())

#Ignore passive fiber forces in muscles
modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())

#Rescale active fiber force curve in muscles. This does seem to improve
#performance for gait motions
modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

#rescale isometric forces just for fun
modelProcessor.append(osim.ModOpScaleMaxIsometricForce(2))

#Add some reserve actuators to help problem
modelProcessor.append(osim.ModOpAddReserves(1))

#Process model for further edits
osimModel = modelProcessor.process()

#Update muscle contraction velocity
for muscleInd in range(osimModel.getMuscles().getSize()):
    osimModel.getMuscles().get(muscleInd).set_max_contraction_velocity(25)
    
#Finalise model
osimModel.finalizeConnections()

#Create a tracking tool
track = osim.MocoTrack()
track.setName('coordinateTracking')

#Set the model in tracking tool
track.setModel(osim.ModelProcessor(osimModel))

#Set coordinate data from IK converted to states in tracking problem
track.setStatesReference(osim.TableProcessor('coordinates.sto'))
track.set_states_global_tracking_weight(1)

#Ensure that any unused data in trc file can be handled
track.set_allow_unused_references(True)

#Since there is only coordinate position data in the states references,
#this setting is enabled to fill in the missing coordinate speed data using
#the derivative of splined position data.
track.set_track_reference_position_derivatives(True)

#Set tracked states in initial guess
track.set_apply_tracked_states_to_guess(True)

#Set state weights
##### TODO: consider individual state weights

#Set times in tracking tool
track.set_initial_time(t0)
track.set_final_time(t1)
# track.set_mesh_interval(0.02)  ### TODO: could solve on a coarser grid to begin with??? Time interval set with solver mesh intervals later...

#Convert to Moco Study to update problem
study = track.initialize()
problem = study.updProblem()

#Update the weight on the default control effort goal
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
effort.setWeight(0.01)

#### TODO: there is an opportunity to set individualised control weights here too...

#Define the solver and set its options
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.set_optim_constraint_tolerance(0.01) #default = 0.01
solver.set_optim_convergence_tolerance(0.01) #default = 0.01
solver.set_num_mesh_intervals(10) ### this is quite coarse to begin with
solver.resetProblem(problem)

########## Bounds --- needs fixing, crashed my Python for some reason?
# set joint RoM as min and max bounds for each free coordinate
# model = modelProcessor.process()
# for i in model.getCoordinateSet():
# 	if i.getMotionType()!=3 and i.get_locked()==False:
# 		name = i.getAbsolutePathString()+'/value'
# 		problem.setStateInfo(name, [i.getRangeMin(), i.getRangeMax()])

#Solve!
solution = study.solve()

#Write to file
solution.write('initialCoordinateTracking_solution.sto')

#Remove the tracked markers file
os.remove('coordinateTracking_tracked_states.sto')

"""

Add notes after optimisation finishes...

"""


# %%

# add extra things to the scaled model
model = osim.Model('out_scaled.osim')

# model.set_ForceSet(osim.ForceSet()) # remove all forces
model.set_MarkerSet(osim.MarkerSet()) # remove all markers

state = model.initSystem()
# muscles = model.getMuscles()
# nameMuscles = [i.getName() for i in muscles]
# nMuscles = muscles.getSize()
# coordinates = model.getCoordinateSet()
# nCoordinates = coordinates.getSize()
# nameCoordinates = [i.getName() for i in coordinates]

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
for i in model.getCoordinateSet():
	path = i.getAbsolutePathString()
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
			if 'lumbar' in i.getName():
				CA.setOptimalForce(100)
			else:
				CA.setOptimalForce(1)
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

model.finalizeConnections()

# set static motion as default pose
static = osim.TimeSeriesTable('out_static.mot')
for i in model.getCoordinateSet():
	i.set_default_value(static.getDependentColumn(i.getAbsolutePathString()+'/value').getElt(0,0))


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
model = modelProc.process()
model.initSystem()
forceSet = model.getForceSet()
for i in range(forceSet.getSize()):
    forcePath = forceSet.get(i).getAbsolutePathString()
    if 'pelvis' in forcePath:
        effort.setWeightForControl(forcePath, 10)

# contact tracking goal (weight = 1). Necessary???
contact = osim.MocoContactTrackingGoal('contact_r', 1)
contact.setExternalLoadsFile('setup_extload.xml')
nameContacts = osim.StdVectorString()
nameContacts.append('ground_heel_r');  nameContacts.append('ground_mid1_r')
nameContacts.append('ground_mid2_r');  nameContacts.append('ground_fore1_r')
nameContacts.append('ground_fore2_r'); nameContacts.append('ground_fore3_r')
nameContacts.append('ground_toe1_r');  nameContacts.append('ground_toe2_r')
contact.addContactGroup(nameContacts, 'right')
contact.setProjection('plane')
contact.setProjectionVector(osim.Vec3(0, 0, 1))
problem.addGoal(contact)

########## Solver
solver = study.initCasADiSolver()
# solver.set_parameters_require_initsystem(False)
solver.set_optim_constraint_tolerance(1e-3)
solver.set_optim_convergence_tolerance(1e-3)

study.printToXML('out_tracking.moco')

# trackingSolution = study.solve()
# trackingSolution.write('out_tracking_solution.sto')
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