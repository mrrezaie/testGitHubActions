import opensim as osim
import os

# only one step
t0 = 0.245 # init time
t1 = 0.535 # end time # stride = 1.025 



# add extra things to the scaled model
model = osim.Model('input/out_scaled.osim')
model.initSystem()

model.setName('moco_adjusted')

# store right muscles
muscles = dict()
for muscle in model.getMuscles():
    if muscle.getName().endswith('_r'):
        # muscle.set_ignore_activation_dynamics(True)
        # muscle.set_ignore_tendon_compliance(True)
        muscle.set_max_contraction_velocity(25)
        muscles[muscle.getName()] = muscle.clone()

# model.set_ForceSet(osim.ForceSet()) # remove all forces
model.updForceSet().clearAndDestroy()

# remove left muscles
for ii in muscles.values():
        model.addForce(ii)

# add residual and reserve actuators
for coordinate in model.getCoordinateSet():
    cName = coordinate.getName()
    if coordinate.getMotionType()!=3 and coordinate.get_locked()==False:
        CA = osim.CoordinateActuator()
        CA.setCoordinate(coordinate)
        CA.setMinControl(float('-inf'))
        CA.setMaxControl(float('+inf'))
        if 'pelvis' in cName: 
            CA.setName(cName+'_residual')
            CA.setOptimalForce(1000)
        else: 
            CA.setName(cName+'_reserve')
            if ('lumbar' in cName) or (cName.endswith('_l')):
                CA.setOptimalForce(500) # strong
            else:
                CA.setOptimalForce(1) # weak
        model.addForce(CA)
        # model.addComponent(CA)


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
            'floor': osim.ContactHalfSpace(osim.Vec3([0.5,0,-0.25]), osim.Vec3([0,0,-osim.SimTK_PI/2]),
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
    # model.addComponent(contactForces[i])

# set static motion as default pose
static = osim.TimeSeriesTable('input/out_static.mot')
for i in model.getCoordinateSet():
    i.set_default_value(static.getDependentColumn(i.getAbsolutePathString()+'/value').getElt(0,0))


model.finalizeConnections()
model.finalizeFromProperties()
model.initSystem()  
# model.printToXML('./output/scaled_upd.osim')


# kinematics to state
stateTable = osim.TableProcessor('input/out_ik.mot')
stateTable.append(osim.TabOpLowPassFilter(15))
stateTable.append(osim.TabOpConvertDegreesToRadians())
stateTable.append(osim.TabOpUseAbsoluteStateNames())
stateTable.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds ())
stateTable = stateTable.process(model)
stateTable.trim(t0, t1)
# osim.STOFileAdapter.write(stateTable, './output/state.sto')


# %% Tracking
######################################################################

import opensim as osim
import numpy as np
import os

# goal weights
markerW  = 1
grfW     = 0.1
controlW = 0.1 

# osim.ModelVisualizer.addDirToGeometrySearchPaths('output/Geometry')

track = osim.MocoTrack()
# track.setName('running_track')

# # coordinate tracking
# track.setStatesReference(osim.TableProcessor('output/state.sto'))
# track.set_allow_unused_references(True)
# track.set_states_global_tracking_weight(10)
# track.set_track_reference_position_derivatives(True)
# track.set_apply_tracked_states_to_guess(True)

# marker tracking
# markerfile = osim.TimeSeriesTableVec3('input/exp_markers.trc').flatten()
# tableProc = osim.TableProcessor(markerfile)
# tableProc.append(osim.TabOpLowPassFilter(15))
# track.setMarkersReference(tableProc)
track.setMarkersReferenceFromTRC('input/exp_markers.trc')
track.set_allow_unused_references(True)
track.set_markers_global_tracking_weight(markerW) # weight of MocoMarkerTrackingGoal
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

# model = osim.Model('output/scaled_upd.osim')
# modelProc = osim.ModelProcessor('output/scaled_upd.osim')
modelProc = osim.ModelProcessor(model)
# # modelProc.append(osim.ModOpAddExternalLoads('input/setup_extLoad.xml')) # already exist in contact tracking
modelProc.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProc.append(osim.ModOpIgnoreTendonCompliance())
modelProc.append(osim.ModOpIgnorePassiveFiberForcesDGF())
modelProc.append(osim.ModOpScaleMaxIsometricForce(1.5))
modelProc.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
# # modelProc.append(osim.ModOpUseImplicitTendonComplianceDynamicsDGF()) # tendon is already rigid
modelProc.append(osim.ModOpIgnoreActivationDynamics())
# # modelProc.append(osim.ModOpRemoveMuscles())
# # modelProc.append(osim.ModOpAddReserves(1))

locked = osim.StdVectorString()
for coordinate in model.getCoordinateSet():
    if coordinate.get_locked():
        locked.append(coordinate.getJoint().getName())
modelProc.append(osim.ModOpReplaceJointsWithWelds(locked))

# model2 = modelProc.process()
# model2.finalizeFromProperties()
# model2.printToXML('output/scaled_upd2.osim')

track.setModel(modelProc)
track.set_minimize_control_effort(True)
track.set_control_effort_weight(controlW) # default weight
track.set_initial_time(t0)
track.set_final_time(t1)
# track.set_mesh_interval(0.01) # ???

# track.printToXML('output/track.xml')
study = track.initialize()
problem = study.updProblem()


########## Bounds
# set joint RoM as min and max bounds for each free coordinate
# It is already set by moco, so it's not necessary
for i in model.getCoordinateSet():
    name = i.getAbsolutePathString()
    if i.getMotionType()!=3 and i.get_locked()==False:
        problem.setStateInfo(name+'/value', [i.getRangeMin(), i.getRangeMax()])

problem.setStateInfoPattern('/jointset/.*/speed', [-20, 20])
problem.setStateInfoPattern('/forceset/.*/activation', [0.01, 1])
# problem.setStateInfoPattern('/forceset/.*/normalized_tendon_force', [0, 1.8], [], []);

# # set activation bounds only for muscles (not all forces)
# for i in model.getMuscles():
#   name = i.getAbsolutePathString()
#   problem.setStateInfoPattern(name+'/activation', [0.01, 1])
#   problem.setStateInfoPattern(name+'/fiber_length', [0, 1])
#   problem.setStateInfoPattern(name+'/normalized_tendon_force', [0, 1])

########## Goals
effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
# effort.setWeight(controlW) # the same for track.set_control_effort_weight

# why???
for i in model.getForceSet():
    forcePath = i.getAbsolutePathString()
    if 'pelvis' in forcePath:
        effort.setWeightForControl(forcePath, 10)

# contact tracking goal (weight = 1).
def GRFTrackingGoal(name, weight=1, projectionVector=[0,0,0]):
    contact = osim.MocoContactTrackingGoal(name, weight)
    contact.setExternalLoadsFile('input/setup_extload.xml')
    nameContacts = osim.StdVectorString()
    for i in ['heel_r', 'mid1_r', 'mid2_r', 'fore1_r', 'fore2_r', 'fore3_r', 'toe1_r', 'toe2_r']:
        nameContacts.append('/forceset/ground_'+i)
    contact.addContactGroup(nameContacts, 'right')
    contact.setProjection('vector')
    contact.setProjectionVector(osim.Vec3(projectionVector))
    return contact
# one goal for every axis
contactX = GRFTrackingGoal('contactX_r', weight=grfW, projectionVector=[1,0,0])
contactY = GRFTrackingGoal('contactY_r', weight=grfW, projectionVector=[0,1,0])
contactZ = GRFTrackingGoal('contactZ_r', weight=grfW, projectionVector=[0,0,1])
problem.addGoal(contactX)
problem.addGoal(contactY)
problem.addGoal(contactZ)


########## Solver
# solver = study.initCasADiSolver()
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
# solver.set_verbosity(2)
# solver.set_optim_solver("ipopt")
# solver.set_parameters_require_initsystem(False)
solver.set_num_mesh_intervals(80)
solver.set_optim_constraint_tolerance(1e-2)
solver.set_optim_convergence_tolerance(1e-2)
solver.set_optim_max_iterations(10000)
# solver.set_parallel(0)

# set coordinates value and speed as initial guesses
initGuess = solver.createGuess('bounds')
# stateTable = osim.TimeSeriesTable('output/state.sto')
x = osim.Vector(stateTable.getIndependentColumn())
nx = initGuess.getTime()
for i in model.getCoordinateSet():
    if i.get_locked()==False:
        for j in ['/value', '/speed']:
            name = i.getAbsolutePathString()+j
            y = osim.Vector(stateTable.getDependentColumn(name).to_numpy())
            vector = osim.interpolate(x,y,nx)
            initGuess.setState(name, vector)

# initGuess.write('output/tracking_init_guess.sto')
solver.setGuess(initGuess)

study.set_write_solution(True)
# study.printToXML('./tracking_simulation.xml')

trackingSolution = study.solve()
# trackingSolution.unseal()
# trackingSolution.write('output/tracking_solution.sto')
# study.visualize(trackingSolution)
