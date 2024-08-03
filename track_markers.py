import opensim as osim
import numpy as np
import os

cwd = os.getcwd() # current working directory where the script is located
model_path    = os.path.join(cwd,'input','out_scaled.osim')
static_path   = os.path.join(cwd,'input','out_static.mot')
markers_path  = os.path.join(cwd,'input','exp_markers.trc')
IK_path       = os.path.join(cwd,'input','out_ik.mot')
ExtLoads_path = os.path.join(cwd,'input','setup_extload.xml')
GRF_path      = os.path.join(cwd,'input','exp_grf.mot')
geometries    = os.path.join(cwd,'input','Geometry')

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

# time frames (right stance only)
t0 = 0.245 # init time
t1 = 0.530 # end time # stride = 1.025 

########## model processing
model = osim.Model(model_path)
model.setName('moco_adjusted')

# replace muscles with DeGrooteFregly2016
osim.DeGrooteFregly2016Muscle().replaceMuscles(model)

# adjusted and store the right muscles only
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
osim.ModelFactory().createReserveActuators(model, 1, float('inf')) # 

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

# add external loads for the joint reaction analysis
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
# goals weight
markerW  = 1
# controlW = 0.001 # (default==0.001 in MocoTrack)

track = osim.MocoTrack()
# track.setName('')
track.setModel( osim.ModelProcessor(model))
track.set_initial_time(t0)
track.set_final_time(t1)
track.set_mesh_interval(0.01) # Hermite-Simpson
track.set_minimize_control_effort(True)
# track.set_control_effort_weight(controlW) # (default==0.001 in MocoTrack)


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
# adjust control goal
effort = osim.MocoControlGoal().safeDownCast(problem.updGoal('control_effort'))
# if caring about dynamic consistency, this minimizes residual actuators more than others
effort.setWeightForControlPattern('.*residual', 10)


########## Solver
# solver = study.initCasADiSolver()
solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
solver.resetProblem(problem)
# solver.set_verbosity(2)
# solver.set_optim_solver("ipopt")
# solver.set_parameters_require_initsystem(True)
# solver.set_num_mesh_intervals(30) # adjusted by track.set_mesh_interval()
print('Total number of mesh intervals', solver.get_num_mesh_intervals())
solver.set_optim_constraint_tolerance(1e-3) # IPOPT default
solver.set_optim_convergence_tolerance(1e-5)
solver.set_optim_max_iterations(10000)
# # implicit for inverse; explicit for forward dynamics
# solver.set_multibody_dynamics_mode('explicit') 
# solver.set_optim_finite_difference_scheme('central')
# solver.set_optim_hessian_approximation('exact')
# solver.set_optim_nlp_scaling_method('gradient-based')
# solver.set_optim_mu_strategy('adaptive') # AttributeError: 'MocoCasADiSolver' object has no attribute 'set_optim_mu_strategy'.
# solver.set_parallel(0)

study.printToXML( os.path.join(cwd,'output','tracking_markers_study.xml') )


########## initial guesses
initGuess = solver.createGuess('bounds') # 'random'
n = initGuess.getNumTimes()
initGuess.setStatesTrajectory(stateTable, True, True)
# initGuess.write( os.path.join(cwd,'output','tracking_markers_init_guess.sto') )
solver.setGuess(initGuess)


########## solve
solution = study.solve()
solution.write( os.path.join(cwd,'output','tracking_markers_solution.sto') )
# solution.unseal()
# study.visualize(solution)


########## post-hoc analyses
# solution = osim.MocoTrajectory( os.path.join(cwd,'output','tracking_markers_solution.sto') )


# get joint contact forces
jointLoadTable = osim.analyzeMocoTrajectorySpatialVec(model, solution, ['.*reaction_on_child'])
suffix = ['_mx','_my','_mz', '_fx','_fy','_fz']
osim.STOFileAdapter().write(jointLoadTable.flatten(suffix), os.path.join(cwd,'output','tracking_markers_joint_load_solution.sto') )