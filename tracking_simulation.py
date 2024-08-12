'''
This is a Moco marker tracking simulation

Options:

    torque driven or muscle driven
    with and without contact tracking goal

'''
# type of simulation
torque_driven       = True
contact_tracking    = True
joint_reaction_goal = False

# goals weight
marker_weight  = 1
grf_weight     = 250
control_weight = 0.001 # (default==0.001 in MocoTrack)
# PFJL_weight    = 0.1

# actuators strength
reserve_weak   = 1
reserve_strong = 200 # ID<150Nm
residual       = 2000

if residual <= 1:
    reduce_residuals = False
else:
    reduce_residuals = True
    residuals_weight = 100 # increase the weight of the residuals in control-effort goal

# solver tolerances
constraint_tol  = 1e-5
convergence_tol = 1e-3

# time frames (right stance only)
t0 = 0.245 # init time
t1 = 0.530 # end time # stride = 1.025 
s = 'r' # side

import opensim as osim
import os
import matplotlib.pyplot as plt

cwd = os.getcwd() # current working directory where the script is located
model_path    = os.path.join(cwd,'input','out_scaled.osim')
static_path   = os.path.join(cwd,'input','out_static.mot')
markers_path  = os.path.join(cwd,'input','exp_markers.trc')
IK_path       = os.path.join(cwd,'input','out_ik.mot')
ID_path       = os.path.join(cwd,'input','out_id.sto')
ExtLoads_path = os.path.join(cwd,'input','setup_extload.xml')
GRF_path      = os.path.join(cwd,'input','exp_grf.mot')
geometries    = os.path.join(cwd,'input','Geometry')

osim.Logger.removeFileSink()
osim.ModelVisualizer.addDirToGeometrySearchPaths(geometries)
# osim.Logger.setLevel(6) # [6 5 4 3 2 1 0]
# osim.Logger.setLevelString('Info') # Off Critical Error Warn Info Debug Trace 

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

# adjust coordinate actuators and muscles
if torque_driven:
    model.setName('moco_torque_driven')
    # remove all forces (and groups)
    model.updForceSet().clearAndDestroy() 

else:
    model.setName('moco_muscle_driven')
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

# add the residual and reseve actuators
osim.ModelFactory().createReserveActuators(model, 1, 1) # float('inf')

# adjust the optimal force of the actuators
for force in model.getForceSet():
    if force.getConcreteClassName() == 'CoordinateActuator':
        CA = osim.CoordinateActuator().safeDownCast(force)
        cName  = CA.get_coordinate()
        # residuals (should be low to allow dynamic consistancy) (can also be minimized through Moco control goal)
        if cName.startswith('pelvis'): 
            CA.setName(cName+'_residual')
            CA.setOptimalForce(residual) # N(m) so weak residuals for dynamics consistancy
        # reserve (should be low for coordinates with muscle(s) and high enough for others)
        else: 
            CA.setName(cName+'_reserve')
            if torque_driven:
                CA.setOptimalForce(reserve_strong) # ID < 150Nm
            else:
                if ('lumbar' in cName) or (not cName.endswith(f'_{s}')): # lumbar and the opposite sites
                    CA.setOptimalForce(reserve_strong) # strong reserve; ID < 150Nm
                else: # coordinates with muscles
                    CA.setOptimalForce(reserve_weak) # weak reserve

if contact_tracking:
    # add contact geometries (right foot only)
    ground  = model.getGround()
    calcn = model.getBodySet().get(f'calcn_{s}')
    toes  = model.getBodySet().get(f'toes_{s}')
    contacts = {
        'S1': osim.ContactSphere(0.025, osim.Vec3([0.010,0,-0.005]), calcn, f'heel_{s}'),
        'S2': osim.ContactSphere(0.020, osim.Vec3([0.090,0,-0.025]), calcn, f'mid1_{s}'),
        'S3': osim.ContactSphere(0.020, osim.Vec3([0.070,0,+0.022]), calcn, f'mid2_{s}'),
        'S4': osim.ContactSphere(0.020, osim.Vec3([0.165,0,-0.027]), calcn, f'fore1_{s}'),
        'S5': osim.ContactSphere(0.020, osim.Vec3([0.125,0,+0.035]), calcn, f'fore2_{s}'),
        'S6': osim.ContactSphere(0.020, osim.Vec3([0.040,0,-0.020]), toes,  f'toe1_{s}'),
        'S7': osim.ContactSphere(0.020, osim.Vec3([0.000,0,+0.045]), toes,  f'toe2_{s}'),
        'floor': osim.ContactHalfSpace( osim.Vec3([0.500,0,-0.250]), 
                                        osim.Vec3([0,0,-osim.SimTK_PI/2]), ground, 'floor')}

    for contact in contacts.keys():
        model.addContactGeometry(contacts[contact])

    # add contact forces between ContactHalfSpace (floor) and the ContactSphere(s)
    contactForces = {
        'S1': osim.SmoothSphereHalfSpaceForce(f'floor_heel_{s}',  contacts['S1'], contacts['floor']), 
        'S2': osim.SmoothSphereHalfSpaceForce(f'floor_mid1_{s}',  contacts['S2'], contacts['floor']), 
        'S3': osim.SmoothSphereHalfSpaceForce(f'floor_mid2_{s}',  contacts['S3'], contacts['floor']), 
        'S4': osim.SmoothSphereHalfSpaceForce(f'floor_fore1_{s}', contacts['S4'], contacts['floor']), 
        'S5': osim.SmoothSphereHalfSpaceForce(f'floor_fore2_{s}', contacts['S5'], contacts['floor']), 
        'S6': osim.SmoothSphereHalfSpaceForce(f'floor_toe1_{s}',  contacts['S6'], contacts['floor']),
        'S7': osim.SmoothSphereHalfSpaceForce(f'floor_toe2_{s}',  contacts['S7'], contacts['floor']),
        }

    # adjust the SmoothSphereHalfSpaceForce parameters
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

# adjust mtp joint range of motion
for cName in ['mtp_angle_r', 'mtp_angle_l']:
    coordinate = model.getCoordinateSet().get(cName)
    coordinate.set_range(0, -80*osim.SimTK_DEGREE_TO_RADIAN) # adjust the min range

# adjust patellofemoral joint range of motion
for cName in ['knee_angle_r_beta', 'knee_angle_l_beta']:
    coordinate = model.getCoordinateSet().get(cName)
    coordinate.set_range(0, 0) # adjust the min range
    coordinate.set_range(1, 2.0944) # adjust the max range

# # set static pose as default
# static = osim.TimeSeriesTable(static_path)
# for coordinate in model.getCoordinateSet():
#     cName = coordinate.getAbsolutePathString()
#     value = static.getDependentColumn(cName+'/value').getElt(0,0)
#     coordinate.set_default_value(value)

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
track.set_control_effort_weight(control_weight) # (default==0.001 in MocoTrack)
# track.set_track_reference_position_derivatives(True)

########## marker tracking
track.setMarkersReferenceFromTRC(markers_path)
track.set_allow_unused_references(True)
track.set_markers_global_tracking_weight(marker_weight) # weight of MocoMarkerTrackingGoal
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
# study.set_write_solution(True)
problem = study.updProblem()


########## Bounds
# Moco already adjust the bounds, so it's not mandatory
# significant improvement in convergence time by reducing these bounds close to the real data
# problem.setStateInfoPattern('/jointset/.*/speed', [-15, 15]) # not much significant
# problem.setStateInfoPattern('.*/knee_angle_.*_beta/value', [0, 2.0944]) # done in model

########## Goals
if contact_tracking:
    # contact tracking goal
    contact = osim.MocoContactTrackingGoal('grf_tracking', grf_weight)
    contact.setExternalLoadsFile(ExtLoads_path)
    nameContactForces = [f'/forceset/floor_heel_{s}',  
                         f'/forceset/floor_mid1_{s}',  f'/forceset/floor_mid2_{s}',  
                         f'/forceset/floor_fore1_{s}', f'/forceset/floor_fore2_{s}', 
                         f'/forceset/floor_toe1_{s}',  f'/forceset/floor_toe2_{s}']
    ContactGroup = osim.MocoContactTrackingGoalGroup(nameContactForces, 'right', 
                            [f'/bodyset/toes_{s}']) # why 'toes' is typically used???
    # no need to use projection
    contact.addContactGroup(ContactGroup)
    contact.setNormalizeTrackingError(True)
    problem.addGoal(contact)

if reduce_residuals:
    # adjust control goal
    effort = osim.MocoControlGoal().safeDownCast(problem.updGoal('control_effort'))
    # if caring about dynamic consistency, this minimizes the residual actuation more than others
    effort.setWeightForControlPattern('.*residual', residuals_weight)


if joint_reaction_goal:
    # reaction goal
    PFJLoadGoal = osim.MocoJointReactionGoal('PFPJ_compressive_force', PFJL_weight)
    PFJLoadGoal.setJointPath(f'/jointset/patellofemoral_{s}')
    PFJLoadGoal.setLoadsFrame('child')
    PFJLoadGoal.setExpressedInFramePath(f'/bodyset/patella_{s}') # child frame
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
solver.set_optim_constraint_tolerance(constraint_tol) # 0.01 MocoTrack default
solver.set_optim_convergence_tolerance(convergence_tol) # 0.01 MocoTrack default
solver.set_optim_max_iterations(10000)
# solver.set_minimize_implicit_multibody_accelerations(True)
# solver.set_implicit_multibody_accelerations_weight(1)
# solver.set_minimize_implicit_auxiliary_derivatives(True)
# solver.set_implicit_auxiliary_derivatives_weight(1)
# solver.set_multibody_dynamics_mode('explicit') # explicit implicit
# solver.set_transcription_scheme('hermite-simpson') # trapezoidal
# solver.set_interpolate_control_midpoints(True)
# solver.set_enforce_path_constraint_midpoints(True)
# solver.set_enforce_constraint_derivatives(True)
# solver.set_optim_finite_difference_scheme('forward') # central forward backward
# solver.set_scale_variables_using_bounds(True)
# solver.set_optim_sparsity_detection() # none random initial-guess
# solver.set_optim_hessian_approximation('exact') # exact limited-memory
# # solver.set_optim_nlp_scaling_method('gradient-based')
# # solver.set_optim_mu_strategy('adaptive')
# solver.set_parallel(0)

study.printToXML( os.path.join(cwd,'output','tracking_study.xml') )


########## initial guesses
initGuess = solver.createGuess('bounds') # 'random'
n = initGuess.getNumTimes()
initGuess.setStatesTrajectory(stateTable, True, True)
# for control in initGuess.getControlNames():
#     initGuess.setControl(control, osim.Vector(n, 0.01))
initGuess.write( os.path.join(cwd,'output','tracking_init_guess.sto') )
solver.setGuess(initGuess)


########## solve
solution = study.solve()
solution.write( os.path.join(cwd,'output','tracking_solution.sto') )
# solution.unseal()
# study.visualize(solution)


# %%
########## post-hoc analyses
# solution = osim.MocoTrajectory( os.path.join(cwd,'output','tracking_solution.sto') )

if contact_tracking:
    # get ground reaction forces
    GRFTable = osim.createExternalLoadsTableForGait(model, solution, nameContactForces, [])
    osim.STOFileAdapter().write(GRFTable, os.path.join(cwd,'output','tracking_grf_solution.sto') )

# get joint contact forces
jointLoadTable = osim.analyzeMocoTrajectorySpatialVec(model, solution, ['.*reaction_on_child'])
suffix = ['_mx','_my','_mz', '_fx','_fy','_fz']
osim.STOFileAdapter().write(jointLoadTable.flatten(suffix), os.path.join(cwd,'output','analyze_joint_load.sto') )

# get actuation
actuation = osim.analyzeMocoTrajectory(model, solution, ['.*actuation'])
osim.STOFileAdapter().write(actuation, os.path.join(cwd,'output','analyze_actuation.sto'))


# %% 
########## plots
# plot residuals
times = solution.getTime().to_numpy()
maxAbs = 0
plt.figure(tight_layout=True)
for fName in solution.getControlNames():
    if fName.endswith('_residual'):
        values = solution.getControl(fName).to_numpy() * residual
        plt.plot(times, values, label=fName.split('/')[-1][:-9])
        if max(abs(values)) > maxAbs:
            maxAbs = max(abs(values))
plt.title(f'Residual Actuators\nMaxAbs = {round(maxAbs,5)}')
plt.xlabel('Times (s)')
plt.ylabel('Actuation (N, Nm)')
plt.legend()
plt.savefig(os.path.join(cwd,'output','graph_residuals.png'))

# plot joints angle
# stateTable = osim.TimeSeriesTable(os.path.join(cwd,'output','state.sto'))
cNames = [f'hip_flexion_{s}', f'hip_adduction_{s}', f'hip_rotation_{s}',
          f'knee_angle_{s}',  f'ankle_angle_{s}',   f'subtalar_angle_{s}']
timesState = stateTable.getIndependentColumn()
plt.figure(figsize=(10,6), tight_layout=True)
plt.suptitle('Joints Angle')
for i,cName in enumerate(cNames):
    if cName.startswith('hip'): jName = f'hip_{s}'
    if cName.startswith('knee'): jName = f'walker_knee_{s}'
    if cName.startswith('ankle'): jName = f'ankle_{s}'
    if cName.startswith('subtalar'): jName = f'subtalar_{s}'
    plt.subplot(2,3,i+1)
    valuesState = stateTable.getDependentColumn(f'/jointset/{jName}/{cName}/value').to_numpy()
    plt.plot(timesState, valuesState, lw=2.5, label='IK')
    values = solution.getState(f'/jointset/{jName}/{cName}/value').to_numpy()
    plt.plot(times, values, lw=2.5, label='sim', ls='--')
    plt.title(cName)
    plt.xlabel('Times (s)')
    plt.ylabel('Angle (Radians)')
    plt.legend()
plt.savefig(os.path.join(cwd,'output','graph_joint_angle.png'))

# plot joints moment
if torque_driven:
    IDExp  = osim.TimeSeriesTable(ID_path)
    idx_t0 = IDExp.getNearestRowIndexForTime(t0)
    idx_t1 = IDExp.getNearestRowIndexForTime(t1)
    IDExp.trimToIndices(idx_t0, idx_t1) # more robust to rounding error
    timesID = IDExp.getIndependentColumn()
    plt.figure(figsize=(10,6), tight_layout=True)
    plt.suptitle('Joints Moment')
    for i,cName in enumerate(cNames):
        plt.subplot(2,3,i+1)
        valuesID = IDExp.getDependentColumn(f'{cName}_moment').to_numpy()
        plt.plot(timesID, valuesID, lw=2.5, label='ID')
        values = solution.getControl(f'/forceset/{cName}_reserve').to_numpy()*reserve_strong
        plt.plot(times, values, lw=2.5, label='sim', ls='--')
        plt.title(cName)
        plt.xlabel('Times (s)')
        plt.ylabel('Moment (Nm)')
        plt.legend()
    plt.savefig(os.path.join(cwd,'output','graph_joint_moment.png'))

# plot GRF
if contact_tracking:
    # GRFTable = osim.TimeSeriesTable(os.path.join(cwd,'output','tracking_grf_solution.sto'))
    times  = GRFTable.getIndependentColumn()
    GRFExp = osim.TimeSeriesTable(GRF_path)
    idx_t0 = GRFExp.getNearestRowIndexForTime(t0)
    idx_t1 = GRFExp.getNearestRowIndexForTime(t1)
    GRFExp.trimToIndices(idx_t0, idx_t1) # more robust to rounding error
    timesExp = GRFExp.getIndependentColumn()
    plt.figure(figsize=(10,3.5), tight_layout=True)
    plt.suptitle('Ground Reaction Forces')
    for i,xyz in enumerate(['x','y','z']):
        plt.subplot(1,3,i+1)
        valuesExp = GRFExp.getDependentColumn(f'ground_force_{s}_v{xyz}').to_numpy()
        plt.plot(timesExp, valuesExp, lw=2.5, label='exp')
        values = GRFTable.getDependentColumn(f'ground_force_{s}_v{xyz}').to_numpy()
        plt.plot(times, values, lw=2.5, label='track', ls='--')
        plt.title(f'F{xyz.upper()}')
        plt.xlabel('Times (s)')
        plt.ylabel('Force (N)')
        plt.legend()
    plt.savefig(os.path.join(cwd,'output','graph_grf.png'))


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
