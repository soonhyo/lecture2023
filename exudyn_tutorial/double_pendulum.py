import exudyn as exu
from exudyn.utilities import *
from exudyn.plot import PlotSensor

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
inertia_link_1 = [[1,0,0],
                  [0,1,0],
                  [0,0,1]]
inertia_link_2 = [[1,0,0],
                  [0,1,0],
                  [0,0,1]]

# Position and orientation parameters
ground_clearance = 0.2
position_link_1 = [0,0,length_link_1 + length_link_2 + ground_clearance]
rotation_matrix_link_1 = RotXYZ2RotationMatrix([pi/2,0,0])
position_link_2 = [0,-length_link_1,length_link_1 + length_link_2 + ground_clearance]
rotation_matrix_link_2 = RotXYZ2RotationMatrix([pi/2,0,0])

# Add world ground object for reference
graphics_ground = GraphicsDataCheckerBoard(point = [0,0,0], size = 4)
object_ground = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                           visualization = VObjectGround(graphicsData = [graphics_ground])))

# Set up first link
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

# Set up second link
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
marker_ground = mbs.AddMarker(MarkerBodyRigid(bodyNumber = object_ground, localPosition = position_link_1))
marker_link_1_A = mbs.AddMarker(MarkerBodyRigid(bodyNumber = body_link_1))
marker_link_1_B = mbs.AddMarker(MarkerBodyRigid(bodyNumber = body_link_1, localPosition = [0,0,length_link_1]))
marker_link_2_A = mbs.AddMarker(MarkerBodyRigid(bodyNumber = body_link_2))

# Add joint between the ground object and the first link
mbs.AddObject(GenericJoint(markerNumbers = [marker_ground, marker_link_1_A],
                           constrainedAxes = [1,1,1,0,1,1],
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