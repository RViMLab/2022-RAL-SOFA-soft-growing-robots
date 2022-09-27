import Sofa
import Sofa.Core as SC
from path import Path

def CatheterNode(parent, name="catheter", gridSize: list = [1, 4, 152], collgridSize: list =[1, 1, 500], scale: list = [1, 1, 1],
                 translation: list = [0, 0, 0], rotation: list = [0, 0, 0], stiffness: float = 8e10,
                 damping: float = 4e3, totalMass:float = 3.14e-3, createTendon:bool =True,
                 tendonValues:dict = {'mass': 3.14e-4, 'linesStiffness': 2e9, 'linesDamping': 40, 'restScale': 1.0}):

    """
    This function creates a node for a steerable catheter with or without a tendon. The variables that can be manually
    set by the user are the following:
        parent: Sofa Node representing the 'root' node of the scene
        name: The name of the new node (default: 'catheter')
        gridSize: Number of bins on the SparseGridTopology of the physical object (default: [1,4,152])
        collgridSize: Number of bins on the SparseGridTopology of the collision object (default: [1,1,500]
        scale: The scale of the catheter for every direction, x and y scale will change the radius while, z will change the length
        translation (mm): New position for the catheter (default: [0,0,0])
        rotation (degree): New rotation for the catheter (default: [0,0,0])
        stiffness (Pa): Stiffness value for the MeshSpringForceField (default: 8e10)
        damping: Damping value for the MeshSpringForceField (default: 4e3)
        totalMass (kg): Mass of the catheter (default: 3.14e-3)
        createTendon: Boolean to add a steerable tendon on the catheter (default: True)
        tendonValues: Dictionary containing the physical parameters of the tendond
            mass: Mass of the tendon
            linesStiffness: Stiffness of the MeshSringForceField of tendon
            linesDamping: Linear damping of the MeshSpringForceField
            restScale: restScale value of the tendon, represents the steering of the catheter
     """

    #assert len(gridSize) == 3 & len(collgridSize) == 3, "The list for the gridSize must be of length 3"
    #assert len(translation) == 3 & len(rotation) == 3 & len(scale) == 3, "The list for the following list must be 3"
    catheter = parent.createChild("catheter")
    catheter.addObject('EulerImplicitSolver', name='odesolver', rayleighMass=0.1, rayleighStiffness=0.1)
    catheter.addObject('CGLinearSolver', iterations=30, tolerance=1e-5, threshold=1e-5)

    catheter.addObject('SparseGridTopology', name='sp_topology', n=gridSize,
                       fileTopology='meshes\Catheter\catheter_v1_3.obj')
    catheter.addObject('MechanicalObject', name='catheter_mobject', topology='@sp_topology', dx=translation[0], dy=translation[1],
                        dz=translation[2], rx=rotation[0], ry=rotation[1], rz=rotation[2], sx=scale[0], sy=scale[1], sz=scale[2])
    catheter.addObject('UniformMass', totalMass=totalMass, topology='@sp_topology')
    catheter.addObject('MeshSpringForceField', name='Springs', stiffness=stiffness, damping=damping)
    catheter.addObject('FixedConstraint', name='FixedConstraint', indices=[0, 556, 429, 985])

    catheter.addObject('UncoupledConstraintCorrection')

    visual2 = catheter.addChild("Visu")

    visual2.addObject('MeshObjLoader', name='mesh_Loader_0', filename='meshes\Catheter\catheter_v1_3.obj')
    visual2.addObject('OglModel', name='Visual', src='@mesh_Loader_0', color='red')
    visual2.addObject('BarycentricMapping', input='@..', output='@Visual')

    if createTendon:
            
        tendon = catheter.addChild("tendon")
        tendon.addObject('EdgeSetTopologyContainer', name='tendon_container', position='@../sp_topology.position',
                        edges=[500, 555, 1056, 1111])
        tendon.addObject('MechanicalObject', name='tendon_mobject', topology='@tendon_container', restScale=tendonValues["restScale"])
        tendon.addObject('UniformMass', totalMass=tendonValues["mass"], topology='@tendon_container')
        tendon.addObject('MeshSpringForceField', name='tendon_springs', topology='@tendon_container', drawMode=4,
                        template='Vec3d', linesStiffness=tendonValues["linesStiffness"],
                        linesDamping=tendonValues["linesDamping"], drawSpringSize=20, noCompression=True)

    cath_surface = catheter.addChild('cath_surface')
    cath_surface.addObject('SparseGridTopology', name='cath_loader', n=collgridSize,
                           fileTopology='meshes\Catheter\catheter_v1_3_cover.obj')
    cath_surface.addObject('MechanicalObject', src='@cath_loader')
    cath_surface.addObject('SphereCollisionModel', simulated=1, moving=1, selfCollision=0,
                           proximity=0, radius=0.95, contactStiffness=1e4, contactFriction=0,
                           contactRestitution=0)
    cath_surface.addObject('BarycentricMapping', input='@..', output='@.')

    return catheter
