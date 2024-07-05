import opensim as osim
import os

model_fileName =    './input/out_scaled.osim'
static_fileName =   './input/out_static.mot'
markers_fileName =  './input/exp_markers.trc'
IK_fileName =       './input/out_ik.mot'
ExtLoads_fileName = './input/setup_extLoad.xml'
GRF_fileName =      './input/exp_grf.mot'
geometries =        './input/Geometry'

osim.ModelVisualizer.addDirToGeometrySearchPaths(geometries)

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
t0 = 0.240 # init time
t1 = 0.540 # end time # stride = 1.025 

########## model processing
model = osim.Model(model_fileName)
model.setName('moco_adjusted')

# replace muscles with DeGrooteFregly2016
osim.DeGrooteFregly2016Muscle().replaceMuscles(model)

# # useful functions
# osim.ModelFactory().removeMuscles(model)
# osim.ModelFactory().replaceJointWithWeldJoint(model, 'mtp_r')
# osim.ModelFactory().replaceJointWithWeldJoint(model, 'mtp_l')
# osim.ModelFactory().replaceMusclesWithPathActuators(model)

# fNames   = [force.getName() for force in model.getForceSet()]
# fClasses = [force.getConcreteClassName() for force in model.getForceSet()]
# for fName, fClass in zip(fNames,fClasses):
#     if ('Muscle' in fClass) and fName.endswith('_r'):
#         muscle = model.getMuscles().get(fName)
#         muscle = osim.DeGrooteFregly2016Muscle().safeDownCast(muscle)
#         muscle.setMinControl(0) # more physiological
#         muscle.set_ignore_activation_dynamics(True)
#         muscle.set_ignore_tendon_compliance(True)
#         muscle.set_ignore_passive_fiber_force(True)
#         muscle.set_active_force_width_scale(1.5)
#         muscle.set_max_contraction_velocity(15) # more physiological
#     else:
#         fIndex = model.getForceSet().getIndex(fName)
#         model.getForceSet().remove(fIndex)

# adjusted and store the right muscles only
muscles = dict()
for muscle in model.getMuscles():
    mName = muscle.getName()
    if mName.endswith('_r'):
        muscle = osim.DeGrooteFregly2016Muscle().safeDownCast(muscle)
        # muscle.setMinControl(0) # more physiological (slower)
        muscle.set_ignore_activation_dynamics(True)
        muscle.set_ignore_tendon_compliance(True)
        muscle.set_ignore_passive_fiber_force(True)
        muscle.set_active_force_width_scale(1.5) # (slower)
        muscle.set_max_contraction_velocity(25) # more physiological
        muscles[mName] = muscle.clone()

# remove all forces (and groups)
model.updForceSet().clearAndDestroy()

# include right muscles only
for muscle in muscles.values():
    model.addForce(muscle)

# # or use this way
# indx = model.getForceSet().getIndex(name)
# model.getForceSet().remove(indx)

# add coordinate actuators
osim.ModelFactory().createReserveActuators(model, 1, float('inf'))

# adjust the optimal force of the actuators
for force in model.getForceSet():
    if force.getConcreteClassName() == 'CoordinateActuator':
        CA = osim.CoordinateActuator().safeDownCast(force)
        cName  = CA.get_coordinate()
        # residuals
        if cName.startswith('pelvis'): 
            CA.setName(cName+'_residual')
            if cName[-3:] in ['_tx','_ty','_tz']:
                CA.setOptimalForce(100) # weak residual
            else:
                CA.setOptimalForce(10) # weak residual
        # reserve
        else: 
            CA.setName(cName+'_reserve')
            if ('lumbar' in cName) or (cName.endswith('_l')):
                CA.setOptimalForce(250) # strong reserve
            else:
                CA.setOptimalForce(1) # weak reserve

# add contact geometries (right foot only)
ground  = model.getGround()
calcn_r = model.getBodySet().get('calcn_r')
toes_r  = model.getBodySet().get('toes_r')
pi = osim.SimTK_PI
contacts = {
    'S1': osim.ContactSphere(0.030, osim.Vec3([0.02,0,0]),          calcn_r, 'heel_r'),
    'S2': osim.ContactSphere(0.025, osim.Vec3([0.1,-0.001,-0.02]),  calcn_r, 'mid1_r'),
    'S3': osim.ContactSphere(0.025, osim.Vec3([0.09,-0.001,0.02]),  calcn_r, 'mid2_r'),
    'S4': osim.ContactSphere(0.025, osim.Vec3([0.18,-0.001,-0.03]), calcn_r, 'fore1_r'),
    'S5': osim.ContactSphere(0.025, osim.Vec3([0.16,-0.001,0]),     calcn_r, 'fore2_r'),
    'S6': osim.ContactSphere(0.025, osim.Vec3([0.14,-0.001,0.03]),  calcn_r, 'fore3_r'),
    'S7': osim.ContactSphere(0.025, osim.Vec3([0.05,-0.001,-0.01]), toes_r,  'toe1_r'),
    'S8': osim.ContactSphere(0.025, osim.Vec3([0.01,-0.001,0.03]),  toes_r,  'toe2_r'),
    'floor': osim.ContactHalfSpace(osim.Vec3([0.5,0,-0.25]), osim.Vec3([0,0,-pi/2]), ground, 'ground')}

for contact in contacts.keys():
    model.addContactGeometry(contacts[contact])

# add contact forces (right foot only)
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
    contactForces[contactForce].set_viscous_friction(0.5)
    contactForces[contactForce].set_transition_velocity(0.2)
    contactForces[contactForce].set_constant_contact_force(1e-5)
    contactForces[contactForce].set_hertz_smoothing(300)
    contactForces[contactForce].set_hunt_crossley_smoothing(50)
    contactForces[contactForce].set_hunt_crossley_smoothing(50)
    model.addForce(contactForces[contactForce])
    # model.addComponent(contactForces[contactForce])

# set static pose as default
static = osim.TimeSeriesTable(static_fileName)
for coordinate in model.getCoordinateSet():
    cName = coordinate.getAbsolutePathString()
    coordinate.set_default_value(static.getDependentColumn(cName+'/value').getElt(0,0))

# finalize the model and write it
model.finalizeConnections()
model.finalizeFromProperties()
state = model.initSystem()  

model.printToXML('./output/scaled_upd.osim')

# modelProc = osim.ModelProcessor(model)
# modelProc.append( osim.ModOpAddExternalLoads(ExtLoads_fileName))
# modelProc.append( osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
# modelProc.append( osim.ModOpIgnoreTendonCompliance())
# modelProc.append( osim.ModOpIgnoreActivationDynamics())
# modelProc.append( osim.ModOpIgnorePassiveFiberForcesDGF())
# modelProc.append( osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
# modelProc.append( osim.ModOpAddExternalLoads(ExtLoads_fileName)) # contact tracking
# modelProc.append( osim.ModOpScaleMaxIsometricForce(1.5))
# modelProc.append( osim.ModOpUseImplicitTendonComplianceDynamicsDGF())
# modelProc.append( osim.ModOpRemoveMuscles())
# modelProc.append( osim.ModOpAddReserves(1))
# modelProc.append( osim.ModOpReplaceJointsWithWelds(['mtp_r','mtp_l']))


########## create state from kinematics
stateTable = osim.TableProcessor(IK_fileName)
stateTable.append(osim.TabOpLowPassFilter(15))
stateTable.append(osim.TabOpConvertDegreesToRadians())
stateTable.append(osim.TabOpUseAbsoluteStateNames())
stateTable.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds ())
stateTable = stateTable.process(model)
stateTable.trim(t0, t1)
osim.STOFileAdapter.write(stateTable, './output/state.sto')

########## Moco tracking simulation
# goals weight
markerW  = 1
GRFW     = 1
controlW = 0.001 # default in MocoTrack
# PFJLW    = 1

track = osim.MocoTrack()
# track.setName('running_track')
track.setModel( osim.ModelProcessor(model))
track.set_minimize_control_effort(True)
track.set_control_effort_weight(controlW)
track.set_initial_time(t0)
track.set_final_time(t1)
track.set_mesh_interval(0.01) # 100 Hz


########## coordinate tracking
# track.setStatesReference(osim.TableProcessor('./output/state.sto'))
# track.set_allow_unused_references(True)
# track.set_states_global_tracking_weight(10)
# track.set_track_reference_position_derivatives(True)
# track.set_apply_tracked_states_to_guess(True)


########## marker tracking
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
markerWeights.cloneAndAppend( osim.MocoWeight('R.ASIS',     8))
markerWeights.cloneAndAppend( osim.MocoWeight('L.ASIS',     8))
markerWeights.cloneAndAppend( osim.MocoWeight('R.PSIS',     8))
markerWeights.cloneAndAppend( osim.MocoWeight('L.PSIS',     8))
markerWeights.cloneAndAppend( osim.MocoWeight('S2',         8))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Knee',     4))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Knee',     4))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Ankle',    4))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Ankle',    4))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Heel',     6))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Heel',     6))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Toe',      6))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Toe',      6))
markerWeights.cloneAndAppend( osim.MocoWeight('R.MT5',      6))
markerWeights.cloneAndAppend( osim.MocoWeight('L.MT5',      6))
track.set_markers_weight_set(markerWeights)

study = track.initialize()
study.set_write_solution(True)
problem = study.updProblem()


########## Bounds
# already set by moco, so it's not necessary


########## Goals
# effort = osim.MocoControlGoal.safeDownCast(problem.updGoal('control_effort'))
# # effort.setWeight(controlW) # the same for track.set_control_effort_weight
# for i in model.getForceSet():
#     forcePath = i.getAbsolutePathString()
#     if 'pelvis' in forcePath:
#         effort.setWeightForControl(forcePath, 10) # why???

# contact tracking goal
contact = osim.MocoContactTrackingGoal('GRF_tracking', GRFW)
contact.setExternalLoadsFile(ExtLoads_fileName)
nameContactForces = ['/forceset/ground_heel_r',  '/forceset/ground_mid1_r', 
                     '/forceset/ground_mid2_r',  '/forceset/ground_fore1_r', 
                     '/forceset/ground_fore2_r', '/forceset/ground_fore3_r', 
                     '/forceset/ground_toe1_r',  '/forceset/ground_toe2_r']
ContactGroup = osim.MocoContactTrackingGoalGroup(nameContactForces, 'right', 
                        ['/bodyset/toes_r']) # why 'toes' is typically used???
# no need to use projection
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
# solver.set_parameters_require_initsystem(True)
# solver.set_num_mesh_intervals(30) # adjusted by track.set_mesh_interval()
print('Total number of mesh intervals', solver.get_num_mesh_intervals())
solver.set_optim_constraint_tolerance(1e-4) # IPOPT default
solver.set_optim_convergence_tolerance(1e-5)
solver.set_optim_max_iterations(10000)
# # implicit for inverse; explicit for forward dynamics
# solver.set_multibody_dynamics_mode('explicit') 
# solver.set_optim_finite_difference_scheme('central')
# solver.set_parallel(0)


########## initial guesses
initGuess = solver.createGuess('bounds')
# stateTable = osim.TimeSeriesTable('./output/state.sto')
x = osim.Vector(stateTable.getIndependentColumn())
nx = initGuess.getTime()
for coordinate in model.getCoordinateSet():
    if not coordinate.get_locked():
        for i in ['/value', '/speed']:
            name = coordinate.getAbsolutePathString()+i
            y = osim.Vector(stateTable.getDependentColumn(name).to_numpy())
            vector = osim.interpolate(x,y,nx)
            initGuess.setState(name, vector)

# initGuess.write('./output/tracking_init_guess.sto')
solver.setGuess(initGuess)

study.printToXML('./output/tracking_study.xml')

trackingSolution = study.solve()
trackingSolution.write('./output/tracking_solution.sto')
# trackingSolution.unseal()
# study.visualize(trackingSolution)

# trackingSolution = osim.MocoTrajectory('./output/tracking_solution.sto')


########## post-hoc analyses

# get ground reaction forces
GRFTable = osim.createExternalLoadsTableForGait(model, trackingSolution, nameContactForces, [])
osim.STOFileAdapter().write(GRFTable, './output/tracking_grf_solution.sto')

# add external loads for the joint reaction analysis
model.addComponent( osim.ExternalLoads(ExtLoads_fileName,True) )
model.initSystem()

# get joint contact forces
jointLoadTable = osim.analyzeMocoTrajectorySpatialVec(model, trackingSolution, ['.*reaction_on_child'])
suffix = ['_mx','_my','_mz', '_fx','_fy','_fz']
osim.STOFileAdapter().write(jointLoadTable.flatten(suffix), './output/tracking_joint_load_solution.sto')

# # remove external loads to avoid further issues related to path
# model.upd_ComponentSet().clearAndDestroy()
