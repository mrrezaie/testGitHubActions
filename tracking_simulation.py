'''
This is a Moco tracking simulation

Options:

    torque driven or muscle driven
    with and without contact tracking goal

'''
# type of simulation
torque_driven       = True
contact_tracking    = False
joint_reaction_goal = False

# goals weight
markerW  = 1
GRFW     = 1
controlW = 1 # (default==0.001 in MocoTrack)
PFJLW    = 0.1

import opensim as osim
import os

cwd = os.getcwd() # current working directory where the script is located
model_path    = os.path.join(cwd,'input','out_scaled.osim')
static_path   = os.path.join(cwd,'input','out_static.mot')
markers_path  = os.path.join(cwd,'input','exp_markers.trc')
IK_path       = os.path.join(cwd,'input','out_ik.mot')
ExtLoads_path = os.path.join(cwd,'input','setup_extload.xml')
GRF_path      = os.path.join(cwd,'input','exp_grf.mot')
geometries    = os.path.join(cwd,'input','Geometry')

# time frames (right stance only)
t0 = 0.245 # init time
t1 = 0.530 # end time # stride = 1.025 

osim.ModelVisualizer.addDirToGeometrySearchPaths(geometries)

# update the path to the GRF STO file in the external loads XML file
ExtLoads = osim.ExternalLoads(ExtLoads_path, True)
ExtLoads.setDataFileName(os.path.abspath(GRF_path))
ExtLoads.printToXML(ExtLoads_path)

# print(cwd)
# print(os.listdir())

# create output directory if it does'nt exist
if not os.path.exists( os.path.join(cwd,'output') ):
    os.mkdir( os.path.join(cwd,'output') )


########## model processing
model = osim.Model(model_path)
model.setName('moco_adjusted')

if torque_driven: # torque driven simulation
    print('A torque driven model')

    # remove all forces (and groups)
    model.updForceSet().clearAndDestroy()

    # add strong coordinate actuators
    osim.ModelFactory().createReserveActuators(model, 2000, 1) # float('inf')
    # rename the actuators
    for force in model.getForceSet():
        if force.getConcreteClassName() == 'CoordinateActuator':
            CA = osim.CoordinateActuator().safeDownCast(force)
            cName  = CA.get_coordinate()
            if cName.startswith('pelvis'): 
                CA.setName(cName+'_residual')
            else: 
                CA.setName(cName+'_reserve')

else: # Muscle driven simulation
    print('A muscle driven model')

    # replace muscles with DeGrooteFregly2016
    osim.DeGrooteFregly2016Muscle().replaceMuscles(model)

    # adjust and store the right muscles only
    muscles = dict()
    for muscle in model.getMuscles():
        mName = muscle.getName()
        if mName.endswith('_r'):
            muscle = osim.DeGrooteFregly2016Muscle().safeDownCast(muscle)
            muscle.setMinControl(0.01) # less physiological but helps convergence
            muscle.set_fiber_damping(0.01) # less physiological but helps convergence
            muscle.set_ignore_activation_dynamics(True)
            muscle.set_ignore_tendon_compliance(True)
            # muscle.set_tendon_compliance_dynamics_mode('implicit')
            muscle.set_ignore_passive_fiber_force(True)
            muscle.set_active_force_width_scale(1.5) # less physiological but helps convergence
            muscle.set_max_contraction_velocity(25)
            MIF = muscle.get_max_isometric_force()
            muscle.set_max_isometric_force(1.5 * MIF) # 1.5 times stronger
            muscles[mName] = muscle.clone()

    # remove all forces (and groups)
    model.updForceSet().clearAndDestroy()

    # include right muscles only
    for muscle in muscles.values():
        model.addForce(muscle)

    # # or remove unwanted forces from ForceSet
    # indx = model.getForceSet().getIndex(name)
    # model.getForceSet().remove(indx)

    # add coordinate actuators
    osim.ModelFactory().createReserveActuators(model, 1, 1) # float('inf')

    # adjust the optimal force of the actuators
    for force in model.getForceSet():
        if force.getConcreteClassName() == 'CoordinateActuator':
            CA = osim.CoordinateActuator().safeDownCast(force)
            cName  = CA.get_coordinate()
            # residuals (should be low to allow dynamic consistancy)
            # will be minimized through Moco control goal
            if cName.startswith('pelvis'): 
                CA.setName(cName+'_residual')
                CA.setOptimalForce(2000)
            # reserve (should be low for coordinates with muscle(s))
            else: 
                CA.setName(cName+'_reserve')
                if ('lumbar' in cName) or (cName.endswith('_l')):
                    CA.setOptimalForce(1000) # strong reserve
                else: # coordinates with muscles
                    CA.setOptimalForce(1) # weak reserve


if contact_tracking:

    # add contact geometries (right foot only)
    ground  = model.getGround()
    calcn_r = model.getBodySet().get('calcn_r')
    toes_r  = model.getBodySet().get('toes_r')
    pi = osim.SimTK_PI
    contacts = {
        'S1': osim.ContactSphere(0.030, osim.Vec3([0.01, 0.000,-0.003]),  calcn_r, 'heel_r'),
        'S2': osim.ContactSphere(0.025, osim.Vec3([0.10,-0.002,-0.021]),  calcn_r, 'mid1_r'),
        'S3': osim.ContactSphere(0.025, osim.Vec3([0.08,-0.002,+0.021]),  calcn_r, 'mid2_r'),
        'S4': osim.ContactSphere(0.025, osim.Vec3([0.17,-0.002,-0.022]),  calcn_r, 'fore1_r'),
        'S5': osim.ContactSphere(0.025, osim.Vec3([0.13,-0.002,+0.032]),  calcn_r, 'fore2_r'),
        'S6': osim.ContactSphere(0.020, osim.Vec3([0.05,-0.002, 0.000]),  toes_r,  'toe_r'),
        'floor': osim.ContactHalfSpace( osim.Vec3([0.5,0,-0.25]), osim.Vec3([0,0,-pi/2]), ground, 'floor')}

    for contact in contacts.keys():
        model.addContactGeometry(contacts[contact])

    # add contact forces (right foot only)
    contactForces = {
        'S1': osim.SmoothSphereHalfSpaceForce('floor_heel_r',  contacts['S1'], contacts['floor']), 
        'S2': osim.SmoothSphereHalfSpaceForce('floor_mid1_r',  contacts['S2'], contacts['floor']), 
        'S3': osim.SmoothSphereHalfSpaceForce('floor_mid2_r',  contacts['S3'], contacts['floor']), 
        'S4': osim.SmoothSphereHalfSpaceForce('floor_fore1_r', contacts['S4'], contacts['floor']), 
        'S5': osim.SmoothSphereHalfSpaceForce('floor_fore2_r', contacts['S5'], contacts['floor']), 
        'S6': osim.SmoothSphereHalfSpaceForce('floor_toe_r',   contacts['S6'], contacts['floor'])}

    for contactForce in contactForces.keys():
        contactForces[contactForce].set_stiffness(1e+6)
        contactForces[contactForce].set_dissipation(2)
        contactForces[contactForce].set_static_friction(0.8)
        contactForces[contactForce].set_dynamic_friction(0.8)
        contactForces[contactForce].set_viscous_friction(0.5)
        contactForces[contactForce].set_transition_velocity(0.2)
        contactForces[contactForce].set_constant_contact_force(1e-5)
        contactForces[contactForce].set_hertz_smoothing(300)
        contactForces[contactForce].set_hunt_crossley_smoothing(50)
        model.addForce(contactForces[contactForce])
        # model.addComponent(contactForces[contactForce])

# set static pose as default
static = osim.TimeSeriesTable(static_path)
for coordinate in model.getCoordinateSet():
    cName = coordinate.getAbsolutePathString()
    value = static.getDependentColumn(cName+'/value').getElt(0,0)
    coordinate.set_default_value(value)

# finalize the model and write it
model.finalizeConnections()
model.finalizeFromProperties()
state = model.initSystem()  
model.printToXML( os.path.join(cwd,'output','scaled_upd.osim') )

if not contact_tracking:
    # add external loads
    model.addComponent( osim.ExternalLoads(ExtLoads_path,True) )
    model.initSystem()


########## create state from kinematics
stateTable = osim.TableProcessor(IK_path)
stateTable.append(osim.TabOpLowPassFilter(15))
stateTable.append(osim.TabOpConvertDegreesToRadians())
stateTable.append(osim.TabOpUseAbsoluteStateNames())
# stateTable.append(osim.TabOpAppendCoupledCoordinateValues())
stateTable.append(osim.TabOpAppendCoordinateValueDerivativesAsSpeeds())
stateTable = stateTable.process(model)
idx_t0 = stateTable.getNearestRowIndexForTime(t0)
idx_t1 = stateTable.getNearestRowIndexForTime(t1)
stateTable.trimToIndices(idx_t0, idx_t1) # more robust to rounding error
osim.STOFileAdapter.write(stateTable, os.path.join(cwd,'output','state.sto') )


# %%
########## Moco tracking simulation
track = osim.MocoTrack()
# track.setName('')
track.setModel( osim.ModelProcessor(model))
track.set_initial_time(t0)
track.set_final_time(t1)
track.set_mesh_interval(0.01) # Hermite-Simpson
track.set_minimize_control_effort(True)
track.set_control_effort_weight(controlW) # (default==0.001 in MocoTrack)
# track.set_track_reference_position_derivatives(True)

########## marker tracking
track.setMarkersReferenceFromTRC(markers_path)
track.set_allow_unused_references(True)
track.set_markers_global_tracking_weight(markerW) # weight of MocoMarkerTrackingGoal
markerWeights = osim.MocoWeightSet()
markerWeights.cloneAndAppend( osim.MocoWeight('R.Shoulder', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Shoulder', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Clavicle', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Clavicle', 1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.ASIS',     4))
markerWeights.cloneAndAppend( osim.MocoWeight('L.ASIS',     4))
markerWeights.cloneAndAppend( osim.MocoWeight('R.PSIS',     4))
markerWeights.cloneAndAppend( osim.MocoWeight('L.PSIS',     4))
markerWeights.cloneAndAppend( osim.MocoWeight('S2',         4))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.TH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.TH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.SH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH1',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH2',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('L.SH3',      1))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Knee',     2))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Knee',     2))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Ankle',    2))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Ankle',    2))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Heel',     3))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Heel',     3))
markerWeights.cloneAndAppend( osim.MocoWeight('R.Toe',      3))
markerWeights.cloneAndAppend( osim.MocoWeight('L.Toe',      3))
markerWeights.cloneAndAppend( osim.MocoWeight('R.MT5',      3))
markerWeights.cloneAndAppend( osim.MocoWeight('L.MT5',      3))
track.set_markers_weight_set(markerWeights)

# get the solver
study = track.initialize()
study.set_write_solution(True)
problem = study.updProblem()


########## Bounds
# already set by moco, so it's not necessary


########## Goals

if contact_tracking:
    # contact tracking goal
    contact = osim.MocoContactTrackingGoal('grf_tracking', GRFW)
    contact.setExternalLoadsFile(ExtLoads_path)
    nameContactForces = ['/forceset/floor_heel_r',  '/forceset/floor_mid1_r', 
                         '/forceset/floor_mid2_r',  '/forceset/floor_fore1_r', 
                         '/forceset/floor_fore2_r', '/forceset/floor_toe_r']
    ContactGroup = osim.MocoContactTrackingGoalGroup(nameContactForces, 'right', 
                            ['/bodyset/toes_r']) # why 'toes' is typically used???
    # no need to use projection
    contact.addContactGroup(ContactGroup)
    contact.setNormalizeTrackingError(True)
    problem.addGoal(contact)

# reduce residuals if there is no contact tracking goal
# adjust control goal
effort = osim.MocoControlGoal().safeDownCast(problem.updGoal('control_effort'))
# if caring about dynamic consistency, this minimizes residual actuators more than others
effort.setWeightForControlPattern('.*residual', 100)


if joint_reaction_goal:
    # reaction goal
    PFJLoadGoal = osim.MocoJointReactionGoal('PFPJ_compressive_force', PFJLW)
    PFJLoadGoal.setJointPath('/jointset/patellofemoral_r')
    PFJLoadGoal.setLoadsFrame('child')
    PFJLoadGoal.setExpressedInFramePath('/bodyset/patella_r') # child frame
    PFJLoadGoal.setReactionMeasures(['force-x']) # or All?
    problem.addGoal(PFJLoadGoal)


########## Solver
# solver = study.initCasADiSolver()
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
# solver.set_verbosity(2)
# solver.set_optim_solver("ipopt")
# solver.set_parameters_require_initsystem(True)
# solver.set_num_mesh_intervals(30) # adjusted by track.set_mesh_interval()
print('Total number of mesh intervals', solver.get_num_mesh_intervals())
solver.set_optim_constraint_tolerance(1e-5) # IPOPT default
solver.set_optim_convergence_tolerance(1e-6)
solver.set_optim_max_iterations(10000)
# solver.set_minimize_implicit_multibody_accelerations(True)
# solver.set_implicit_multibody_accelerations_weight(1)
# solver.set_minimize_implicit_auxiliary_derivatives(True)
# solver.set_implicit_auxiliary_derivatives_weight(1e-3)
# solver.set_multibody_dynamics_mode('implicit') # explicit
# solver.set_transcription_scheme('hermite-simpson') # trapezoidal
# solver.set_interpolate_control_midpoints(True)
# solver.set_enforce_path_constraint_midpoints(True)
# solver.set_enforce_constraint_derivatives(True)
solver.set_optim_finite_difference_scheme('backward') # central forward backward
# solver.set_optim_sparsity_detection() # none random initial-guess
# solver.set_optim_hessian_approximation('exact')
# solver.set_optim_nlp_scaling_method('gradient-based')
# solver.set_optim_mu_strategy('adaptive')
# solver.set_parallel(0)

study.printToXML( os.path.join(cwd,'output','tracking_study.xml') )


########## initial guesses
initGuess = solver.createGuess('bounds') # 'random'
n = initGuess.getNumTimes()
initGuess.setStatesTrajectory(stateTable, True, True)
# initGuess.write( os.path.join(cwd,'output','tracking_init_guess.sto') )
solver.setGuess(initGuess)


########## solve
solution = study.solve()
solution.write( os.path.join(cwd,'output','tracking_solution.sto') )
# solution.unseal()
# study.visualize(solution)


########## post-hoc analyses
# solution = osim.MocoTrajectory( os.path.join(cwd,'output','tracking_solution.sto') )

if contact_tracking:
    # get ground reaction forces
    GRFTable = osim.createExternalLoadsTableForGait(model, solution, nameContactForces, [])
    osim.STOFileAdapter().write(GRFTable, os.path.join(cwd,'output','tracking_grf_solution.sto') )

# get joint contact forces
jointLoadTable = osim.analyzeMocoTrajectorySpatialVec(model, solution, ['.*reaction_on_child'])
suffix = ['_mx','_my','_mz', '_fx','_fy','_fz']
osim.STOFileAdapter().write(jointLoadTable.flatten(suffix), os.path.join(cwd,'output','tracking_joint_load_solution.sto') )


# %% 
########## useful but unused 

# # remove external loads to avoid further issues related to path
# model.upd_ComponentSet().clearAndDestroy()

# # useful functions
# osim.ModelFactory().removeMuscles(model)
# osim.ModelFactory().replaceJointWithWeldJoint(model, 'mtp_r')
# osim.ModelFactory().replaceJointWithWeldJoint(model, 'mtp_l')
# osim.ModelFactory().replaceMusclesWithPathActuators(model)

# modelProc = osim.ModelProcessor(model)
# modelProc.append( osim.ModOpAddExternalLoads(ExtLoads_path))
# modelProc.append( osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
# modelProc.append( osim.ModOpIgnoreTendonCompliance())
# modelProc.append( osim.ModOpIgnoreActivationDynamics())
# modelProc.append( osim.ModOpIgnorePassiveFiberForcesDGF())
# modelProc.append( osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
# modelProc.append( osim.ModOpAddExternalLoads(ExtLoads_path)) # contact tracking
# modelProc.append( osim.ModOpScaleMaxIsometricForce(1.5))
# modelProc.append( osim.ModOpUseImplicitTendonComplianceDynamicsDGF())
# modelProc.append( osim.ModOpRemoveMuscles())
# modelProc.append( osim.ModOpAddReserves(1))
# modelProc.append( osim.ModOpReplaceJointsWithWelds(['mtp_r','mtp_l']))
