import exudyn as exu
from exudyn.utilities import *
from exudyn.plot import PlotSensor
from exudyn.FEM import *

import ngsolve as ngs
from netgen.meshing import *
from netgen.csg import *

# Create system container
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# System parameters
gravity = [0,0,-9.81]

# Link parameters
mass_link_1 = 1
mass_link_2 = 5
length_link_1 = 1.0
length_link_2 = 0.5
length_base = 0.3
radius_base = 0.1
inertia_link_1 = [[1,0,0],
                  [0,1,0],
                  [0,0,1]]
inertia_link_2 = [[1,0,0],
                  [0,1,0],
                  [0,0,1]]

# Position and orientation parameters
ground_clearance = 0.2
position_fem_base = [0,0,length_link_1 + length_link_2 + ground_clearance]
rotation_matrix_fem_base = RotXYZ2RotationMatrix([pi/2,pi/2,0])
position_link_1 = [length_base+0.12/2,0,length_link_1 + length_link_2 + ground_clearance]
rotation_matrix_link_1 = RotXYZ2RotationMatrix([pi/2,0,0])
position_link_2 = [length_base+0.12/2,-length_link_1,length_link_1 + length_link_2 + ground_clearance]
rotation_matrix_link_2 = RotXYZ2RotationMatrix([pi/2,0,0])

# Add world ground object for reference
graphics_ground = GraphicsDataCheckerBoard(point = [0,0,0], size = 4)
object_ground = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                           visualization = VObjectGround(graphicsData = [graphics_ground])))

# Parameters for FEM base
number_eigenmodes = 8
rho = 1400 # Density in kg/m³, e.g. PVC: 1400 kg/m³, Steel: 7850 kg/m³
E_modulus = 1e7 # in Pascal, e.g. PVC: 10 MPa = 1e7 Pa, Steel: 210 GPa = 2.1e11 Pa
nu = 0.4 # Possion Ratio, e.g. PVC: 0.4, Steel: 0.3
dampingK = 1e-2 # Rayleigh damping factor, stiffness proportional damping
mesh_order = 2
mesh_size = 0.1

# Create meshed object geometry (here: cylinder)
geo = CSGeometry()
fb = [{'p0':[0,0,0], 'p1':[0,0,length_base], 'axis0':[0,0,1], 'axis1':[0,0,1]}]
p0 = fb[0]['p0']
axis = fb[0]['axis0']
pnt0 = Pnt(p0[0], p0[1], p0[2])
pnt1 = pnt0 + Vec(axis[0]*length_base, axis[1]*length_base, axis[2]*length_base)
cyl = Cylinder(pnt0, pnt1, radius_base)
plane0 = Plane(pnt0, Vec(-axis[0], -axis[1], -axis[2]) )
plane1 = Plane(pnt1, Vec(axis[0], axis[1], axis[2]) )
cylinder = cyl * plane0 * plane1
geo.Add(cylinder)
mesh = ngs.Mesh(geo.GenerateMesh(maxh = mesh_size))
mesh.Curve(1)    

# Create FEM interface
# bfM: mass matrix, bfK: stiffness matrix, fes: finite element space 
fem = FEMinterface()
[bfM, bfK, fes] = fem.ImportMeshFromNGsolve(mesh, density = rho, youngsModulus = E_modulus, 
                                            poissonsRatio = nu, meshOrder = mesh_order)

print("Total number of nodes: ", fem.NumberOfNodes())

# Get the nodes of the interface planes and their weighting
node_plane_0 = fem.GetNodesInPlane(fb[0]['p0'], fb[0]['axis0'])
weights_plane_0 = np.array((1./len(node_plane_0)) * np.ones(len(node_plane_0)))
node_plane_1  = fem.GetNodesInPlane(fb[0]['p1'], fb[0]['axis1'])
weights_plane_1 = np.array((1./len(node_plane_1))*np.ones(len(node_plane_1)))
boundaryList = [node_plane_0, node_plane_1]

# Compute the flexible modes
print("Computing flexible modes... ")
fem.ComputeHurtyCraigBamptonModes(boundaryNodesList = boundaryList, 
                                  nEigenModes = number_eigenmodes, 
                                  useSparseSolver = True,
                                  computationMode = HCBstaticModeSelection.RBE2)

fem.ComputePostProcessingModesNGsolve(fes, material = KirchhoffMaterial(E_modulus, nu, rho),
                                      outputVariableType = exu.OutputVariableType.StressLocal)

# Create the component mode synthesis (CMS) element
cms = ObjectFFRFreducedOrderInterface(fem)

# Add the reduced order model to the mbs
o_FFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef = position_fem_base, 
                                       initialVelocity = [0,0,0],
                                       rotationMatrixRef = rotation_matrix_fem_base,
                                       initialAngularVelocity = [0,0,0],
                                       massProportionalDamping = 0,
                                       stiffnessProportionalDamping = dampingK,
                                       gravity = gravity,
                                       color = [0.1,0.9,0.1,1.],
                                       )

# Create the interface markers of the FEM base
m_base_A = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber = o_FFRF['oFFRFreducedOrder'], 
                                                 meshNodeNumbers = np.array(node_plane_0),
                                                 weightingFactors = weights_plane_0))

m_base_B = mbs.AddMarker(MarkerSuperElementRigid(bodyNumber = o_FFRF['oFFRFreducedOrder'], 
                                                 meshNodeNumbers = np.array(node_plane_1),
                                                 weightingFactors = weights_plane_1))

# Create first link
inertia_link_1 = RigidBodyInertia(mass = mass_link_1,
                                  com = [0,0,length_link_1/2],
                                  inertiaTensor = inertia_link_1)

graphics_link_1 = GraphicsDataRigidLink(p0 = [0,0,0],
                                        p1 = [0,0,length_link_1],
                                        axis0 = [1,0,0],
                                        axis1 = [1,0,0],
                                        radius = [0.08,0.08],
                                        thickness = 0.08,
                                        width = [0.12,0.12],
                                        color = color4orange)

[node_link_1, body_link_1] = AddRigidBody(mainSys = mbs,
                                          inertia = inertia_link_1,
                                          nodeType = exu.NodeType.RotationEulerParameters,
                                          position = position_link_1,
                                          rotationMatrix = rotation_matrix_link_1,
                                          gravity = gravity,
                                          graphicsDataList = [graphics_link_1])

# Create second link
inertia_link_2 = RigidBodyInertia(mass = mass_link_2,
                                  com = [0,0,length_link_2/2],
                                  inertiaTensor = inertia_link_2)

graphics_link_2 = GraphicsDataRigidLink(p0 = [0,0,0],
                                        p1 = [0,0,length_link_2],
                                        axis0 = [1,0,0],
                                        axis1 = [1,0,0],
                                        radius = [0.07,0.08],
                                        thickness = 0.08,
                                        width = [0.13,0.12],
                                        color = color4green)

[node_link_2, body_link_2] = AddRigidBody(mainSys = mbs,
                                          inertia = inertia_link_2,
                                          nodeType = exu.NodeType.RotationEulerParameters,
                                          position = position_link_2,
                                          rotationMatrix = rotation_matrix_link_2,
                                          gravity = gravity,
                                          graphicsDataList = [graphics_link_2])

# Create markers on each body for the interconnection with joints
marker_ground = mbs.AddMarker(MarkerBodyRigid(bodyNumber = object_ground, localPosition = position_fem_base))
marker_link_1_A = mbs.AddMarker(MarkerBodyRigid(bodyNumber = body_link_1, localPosition = [-0.12/2,0,0]))
marker_link_1_B = mbs.AddMarker(MarkerBodyRigid(bodyNumber = body_link_1, localPosition = [0,0,length_link_1]))
marker_link_2_A = mbs.AddMarker(MarkerBodyRigid(bodyNumber = body_link_2))

# Add joint between the ground object and the base link
mbs.AddObject(GenericJoint(markerNumbers = [marker_ground, m_base_A],
                           constrainedAxes = [1,1,1,1,1,1],
                           rotationMarker0 = rotation_matrix_fem_base,
                           visualization = VObjectJointGeneric(axesRadius=0.03, axesLength=0.3)))

# Add joint between the base link and the first link
mbs.AddObject(GenericJoint(markerNumbers = [m_base_B, marker_link_1_A],
                           constrainedAxes = [1,1,1,0,1,1],
                           rotationMarker0 = RotXYZ2RotationMatrix([0,-pi/2,0]),
                           visualization = VObjectJointGeneric(axesRadius=0.03, axesLength=0.3)))

# Add a joint between the first link and the second link
mbs.AddObject(GenericJoint(markerNumbers = [marker_link_1_B, marker_link_2_A],
                           constrainedAxes = [1,1,1,0,1,1],
                           visualization = VObjectJointGeneric(axesRadius=0.03, axesLength=0.3)))

# Add sensors
sensor_link_2_position = mbs.AddSensor(SensorBody(bodyNumber = body_link_2,
                                                  localPosition = [0,0,length_link_2],
                                                  name = "sensor",
                                                  storeInternal = True,
                                                  outputVariableType = exu.OutputVariableType.Position))

# Assemble the multibody system
mbs.Assemble()

# Set the simulation settings
end_time = 10
step_size = 0.01
solution_write_period = 0.01
sensor_write_period = 0.01

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.endTime = end_time
simulationSettings.timeIntegration.numberOfSteps = int(end_time/step_size)
simulationSettings.solutionSettings.solutionWritePeriod = solution_write_period
simulationSettings.solutionSettings.sensorsWritePeriod = sensor_write_period
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = True

# Adjust visualization settings
SC.visualizationSettings.connectors.show = False
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.loads.drawSimplified = False
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.StressLocal
SC.visualizationSettings.contour.outputVariableComponent = -1 # -1 displays the norm

# Open the visualization
exu.StartRenderer()
mbs.WaitForUserToContinue()

# Solve the dynamical system
exu.SolveDynamic(mbs,
                 simulationSettings,
                 solverType = exudyn.DynamicSolverType.GeneralizedAlpha)

# Close the visualization
SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()

# Create plots for the sensor data
PlotSensor(mbs,
           sensorNumbers = sensor_link_2_position,
           components = [0,1,2],
           title = "Link 2 Endpoint World Position")