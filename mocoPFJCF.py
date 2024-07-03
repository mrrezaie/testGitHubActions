import opensim as osim
import os

model_fileName =    './input/out_scaled.osim'
static_fileName =   './input/out_static.mot'
markers_fileName =  './input/exp_markers.trc'
IK_fileName =       './input/out_ik.mot'
ExtLoads_fileName = './input/setup_extLoad.xml'
GRF_fileName =      './input/exp_grf.mot'

# update the path to the GRF STO file in the external loads XML file
ExtLoads = osim.ExternalLoads(ExtLoads_fileName, True)
ExtLoads.setDataFileName(os.path.abspath(GRF_fileName))
ExtLoads.printToXML(ExtLoads_fileName)

# print(os.getcwd())
# print(os.listdir())

# create output directory if it does'nt exist
if not os.path.exists('./output'):
    os.mkdir('./output')

# time frames (only one step)
t0 = 0.245 # init time
t1 = 0.535 # end time # stride = 1.025 

# add extra things to the scaled model
model = osim.Model(model_fileName)
model.initSystem()

model.setName('moco_adjusted')

# store right muscles
muscles = dict()
for muscle in model.getMuscles():
    if muscle.getName().endswith('_r'):
        if muscle.getConcreteClassName() == 'Millard2012EquilibriumMuscle':
            muscle = osim.Millard2012EquilibriumMuscle().safeDownCast(muscle)
            muscle.setMinimumActivation(0)
            muscle.setMinControl(0)
        # muscle.set_ignore_activation_dynamics(True)
        # muscle.set_ignore_tendon_compliance(True)
        muscle.set_max_contraction_velocity(25)
        muscles[muscle.getName()] = muscle.clone()

# remove all forces
model.updForceSet().clearAndDestroy() # model.set_ForceSet(osim.ForceSet())

# include right muscles only and remove left muscles
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


# add contact geometries (only right foot)
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
            'S7': osim.ContactSphere(0.02, osim.Vec3([0.05,-0.001,-0.01]),
                        model.getBodySet().get('toes_r'), 'toe1_r'),
            'S8': osim.ContactSphere(0.02, osim.Vec3([0.01,-0.001,0.03]),
                        model.getBodySet().get('toes_r'), 'toe2_r'),
            'floor': osim.ContactHalfSpace(osim.Vec3([0.5,0,-0.25]), osim.Vec3([0,0,-osim.SimTK_PI/2]),
                              model.getGround(), 'ground')}

for contact in contacts.keys():
    model.addContactGeometry(contacts[contact])

# add contact forces (only right foot)
contactForces = {
            'S1F': osim.SmoothSphereHalfSpaceForce('ground_heel_r',  contacts['S1'], contacts['floor']), 
            'S2F': osim.SmoothSphereHalfSpaceForce('ground_mid1_r',  contacts['S2'], contacts['floor']), 
            'S3F': osim.SmoothSphereHalfSpaceForce('ground_mid2_r',  contacts['S3'], contacts['floor']), 
            'S4F': osim.SmoothSphereHalfSpaceForce('ground_fore1_r', contacts['S4'], contacts['floor']), 
            'S5F': osim.SmoothSphereHalfSpaceForce('ground_fore2_r', contacts['S5'], contacts['floor']), 
            'S6F': osim.SmoothSphereHalfSpaceForce('ground_fore3_r', contacts['S6'], contacts['floor']), 
            'S7F': osim.SmoothSphereHalfSpaceForce('ground_toe1_r',  contacts['S7'], contacts['floor']), 
            'S8F': osim.SmoothSphereHalfSpaceForce('ground_toe2_r',  contacts['S8'], contacts['floor'])}

for contactForce in contactForces.keys():
    contactForces[contactForce].set_stiffness(3067776)
    contactForces[contactForce].set_dissipation(2)
    contactForces[contactForce].set_static_friction(0.8)
    contactForces[contactForce].set_dynamic_friction(0.8)
    contactForces[contactForce].set_hertz_smoothing(300)
    contactForces[contactForce].set_hunt_crossley_smoothing(50)
    contactForces[contactForce].set_viscous_friction(0.5)
    contactForces[contactForce].set_transition_velocity(0.2)
    contactForces[contactForce].set_constant_contact_force(1e-5)
    model.addForce(contactForces[contactForce])
    # model.addComponent(contactForces[contactForce])

# set static pose as default
static = osim.TimeSeriesTable(static_fileName)
for coordinate in model.getCoordinateSet():
    cName = coordinate.getAbsolutePathString()
    coordinate.set_default_value(static.getDependentColumn(cName+'/value').getElt(0,0))

model.finalizeConnections()
model.finalizeFromProperties()
state = model.initSystem()  

# create state from kinematics
stateTable = osim.TableProcessor(IK_fileName)
stateTable.append(osim.TabOpLowPassFilter(15))
stateTable.append(osim.TabOpConvertDegreesToRadians())
stateTable.append(osim.TabOpUseAbsoluteStateNames())
stateTable.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds ())
stateTable = stateTable.process(model)
stateTable.trim(t0, t1)
osim.STOFileAdapter.write(stateTable, './output/state.sto')


# Tracking
######################################################################

import opensim as osim
import numpy as np
import os

# goal weights
markerW  = 1
GRFW     = 1
# PFJLW    = 1
# controlW = 0.1 # use default

# osim.ModelVisualizer.addDirToGeometrySearchPaths('./output/Geometry')

track = osim.MocoTrack()
# track.setName('running_track')

# # coordinate tracking
# track.setStatesReference(osim.TableProcessor('./output/state.sto'))
# track.set_allow_unused_references(True)
# track.set_states_global_tracking_weight(10)
# track.set_track_reference_position_derivatives(True)
# track.set_apply_tracked_states_to_guess(True)

# marker tracking
# markerfile = osim.TimeSeriesTableVec3(markers_fileName).flatten()
# tableProc = osim.TableProcessor(markerfile)
# tableProc.append(osim.TabOpLowPassFilter(15))
# track.setMarkersReference(tableProc)
track.setMarkersReferenceFromTRC(markers_fileName)
track.set_allow_unused_references(True)
track.set_markers_global_tracking_weight(markerW) # weight of MocoMarkerTrackingGoal
markerWeights = osim.MocoWeightSet()
markerWeights.cloneAndAppend( osim.MocoWeight('R.Shoulder', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Shoulder', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Clavicle', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Clavicle', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.ASIS', 10))
markerWeights.cloneAndAppend( osim.MocoWeight('L.ASIS', 10))
markerWeights.cloneAndAppend( osim.MocoWeight('R.PSIS', 10))
markerWeights.cloneAndAppend( osim.MocoWeight('L.PSIS', 10))
markerWeights.cloneAndAppend( osim.MocoWeight('S2', 10))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH1', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH2', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH3', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH1', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH2', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH3', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH1', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH2', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH3', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH1', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH2', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH3', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Knee', 4))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Knee', 4))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Ankle', 4))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Ankle', 4))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Heel', 5))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Heel', 5))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Toe', 5))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Toe', 5))
markerWeights.cloneAndAppend( osim.MocoWeight('R.MT5', 5))
markerWeights.cloneAndAppend( osim.MocoWeight('L.MT5', 5))
track.set_markers_weight_set(markerWeights)

modelProc = osim.ModelProcessor(model)
# modelProc.append( osim.ModOpAddExternalLoads(ExtLoads_fileName)) # already exist in contact tracking
modelProc.append( osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
modelProc.append( osim.ModOpIgnoreTendonCompliance())
modelProc.append( osim.ModOpIgnoreActivationDynamics())
modelProc.append( osim.ModOpIgnorePassiveFiberForcesDGF())
modelProc.append( osim.ModOpScaleMaxIsometricForce(1.5))
modelProc.append( osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
# # modelProc.append( osim.ModOpUseImplicitTendonComplianceDynamicsDGF()) # tendon is already rigid
# # modelProc.append( osim.ModOpRemoveMuscles())
# # modelProc.append( osim.ModOpAddReserves(1))

# # convert mtp joints to weld joints
# modelProc.append( osim.ModOpReplaceJointsWithWelds(['mtp_r','mtp_l']))

track.setModel(modelProc)
track.set_minimize_control_effort(True)
# track.set_control_effort_weight(controlW) # default weight
track.set_initial_time(t0)
track.set_final_time(t1)
track.set_mesh_interval(0.008) # 125 Hz

# track.printToXML('./output/track.xml')
study = track.initialize()
problem = study.updProblem()


########## Bounds
# already set by moco, so it's not necessary

# set joint RoM as min and max bounds for each free coordinate
# for coordinate in model.getCoordinateSet():
#     cName = coordinate.getAbsolutePathString()
#     if not coordinate.isConstraint():
#         problem.setStateInfo(cName+'/value', [coordinate.getRangeMin(), coordinate.getRangeMax()])

# problem.setStateInfoPattern('/jointset/.*/speed', [-20, 20])
# problem.setStateInfoPattern('/forceset/.*/activation', [0.01, 1])
# problem.setStateInfoPattern('/forceset/.*/normalized_tendon_force', [0, 1.8], [], []);

# # set activation bounds only for muscles (not all forces)
# for i in model.getMuscles():
#   name = i.getAbsolutePathString()
#   problem.setStateInfoPattern(name+'/activation', [0.01, 1])
#   problem.setStateInfoPattern(name+'/fiber_length', [0, 1])
#   problem.setStateInfoPattern(name+'/normalized_tendon_force', [1, 1.8])

########## Goals
# effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
# # effort.setWeight(controlW) # the same for track.set_control_effort_weight
# for i in model.getForceSet():
#     forcePath = i.getAbsolutePathString()
#     if 'pelvis' in forcePath:
#         effort.setWeightForControl(forcePath, 10) # why???

# contact tracking goal (weight = 1).
contact = osim.MocoContactTrackingGoal('GRF_tracking', GRFW)
contact.setExternalLoadsFile(ExtLoads_fileName)
nameContactForces = ['/forceset/ground_heel_r',  '/forceset/ground_mid1_r', 
                     '/forceset/ground_mid2_r',  '/forceset/ground_fore1_r', 
                     '/forceset/ground_fore2_r', '/forceset/ground_fore3_r', 
                     '/forceset/ground_toe1_r',  '/forceset/ground_toe2_r']
ContactGroup = osim.MocoContactTrackingGoalGroup(nameContactForces, 'right', 
                                        ['/bodyset/toes_r']) # why toes is typically used???
contact.addContactGroup(ContactGroup)
contact.setNormalizeTrackingError(True)
contact.setEnabled(True)
problem.addGoal(contact)

# # reaction goal
# PFJLoadGoal = osim.MocoJointReactionGoal('PFPJ_compressive_force', PFJLW)
# PFJLoadGoal.setJointPath('/jointset/patellofemoral_r')
# PFJLoadGoal.setLoadsFrame('child')
# PFJLoadGoal.setExpressedInFramePath('/bodyset/patella_r') # child frame
# PFJLoadGoal.setReactionMeasures(['force-x']) # or All?
# problem.addGoal(PFJLoadGoal)

########## Solver
# solver = study.initCasADiSolver()
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
# solver.set_verbosity(2)
# solver.set_optim_solver("ipopt")
# solver.set_parameters_require_initsystem(False)
# solver.set_num_mesh_intervals(30) # adjusted by track.set_mesh_interval()
print('Total number of mesh intervals', solver.get_num_mesh_intervals())
solver.set_optim_constraint_tolerance(1e-4)
solver.set_optim_convergence_tolerance(1e-5)
solver.set_optim_max_iterations(10000)
# solver.set_multibody_dynamics_mode('explicit') # implicit for inverse; explicit for forward dynamics
# solver.set_optim_finite_difference_scheme('central')
# solver.set_parallel(0)

# set coordinates value and speed as initial guesses
initGuess = solver.createGuess('bounds')
# stateTable = osim.TimeSeriesTable('./output/state.sto')
x = osim.Vector(stateTable.getIndependentColumn())
nx = initGuess.getTime()
for coordinate in model.getCoordinateSet():
    if coordinate.get_locked()==False:
        for j in ['/value', '/speed']:
            name = coordinate.getAbsolutePathString()+j
            y = osim.Vector(stateTable.getDependentColumn(name).to_numpy())
            vector = osim.interpolate(x,y,nx)
            initGuess.setState(name, vector)

initGuess.write('./output/tracking_init_guess.sto')
solver.setGuess(initGuess)

study.set_write_solution(True)
study.printToXML('./output/tracking_study.xml')

trackingSolution = study.solve()
trackingSolution.write('./output/tracking_solution.sto')
# trackingSolution.unseal()
# study.visualize(trackingSolution)

# trackingSolution = osim.MocoTrajectory('./output/tracking_solution.sto')

# post-hoc analyses
modelProc.append(osim.ModOpAddExternalLoads(ExtLoads_fileName))
model = modelProc.process()
model.initSystem()
model.printToXML('./output/scaled_upd.osim')

# get ground reaction forces
GRFTable = osim.createExternalLoadsTableForGait(model, trackingSolution, nameContactForces, [])
osim.STOFileAdapter().write(GRFTable, './output/tracking_grf_solution.sto')

# get joint contact forces
jointLoadTable = osim.analyzeMocoTrajectorySpatialVec(model, trackingSolution, ['.*reaction_on_child'])
suffix = ['_force_x','_force_y', '_force_z', '_moment_x','_moment_y', '_moment_z']
osim.STOFileAdapter().write(jointLoadTable.flatten(suffix), './output/tracking_joint_load_solution.sto')
