import Sofa
import Sofa.Core as SC
from path import Path

def SheathNode(parent, name = "Sheath", gridSize: list = [1, 1, 25], collgridSize: list =[1, 1, 62], scale: list = [1, 1, 1],
                translation: list = [1, -3.3, 20], rotation: list = [0, 0, 45], stiffness: float = 3e8,
                damping: float = 4, totalMass:float = 3.14e-3, restScale=1.0, pressure=-7e4):
    
    """
    This function creates a node for teh eversion sheath. The default values are set for the catheter and the sheath to be 
    aligned with each other, based on our CAD Models. The variables that can be manually set by the user are the following:
        parent: Sofa Node representing the 'root' node of the scene
        name: The name of the new node (default: 'Sheath')
        gridSize: Number of bins on the SparseGridTopology of the physical object (default: [1,1,25])
        collgridSize: Number of bins on the SparseGridTopology of the collision object (default: [1,1,62]
        scale: The scale of the sheath for every direction, x and y scale will change the radius while, z will change the length
        translation (mm): New position for the sheath (default: [1,-3.3,20])
        rotation (degree): New rotation for the sheath (default: [0,0,45])
        stiffness (Pa): Stiffness value for the MeshSpringForceField (default: 3e8)
        damping: Damping value for the MeshSpringForceField (default: 4.0)
        totalMass (kg): Mass of the sheath (default: 3.14e-3)
        pressure: The amplitude of the internal pressure field of the sheath (default: -7e4)
        restScale: Initial change on position for the sheath (default: 1.0)
     """

    #assert len(gridSize) == 3 & len(collgridSize) == 3, "The list for the gridSize must be of length 3"
    #assert len(translation) == 3 & len(rotation) == 3 & len(scale) == 3, "The list for the following list must be 3"
    sheath = parent.addChild('Sheath')

    sheath.addObject('EulerImplicitSolver', name='odesolver', rayleighMass=0.001, rayleighStiffness=0.001)
    sheath.addObject('CGLinearSolver', iterations=30, tolerance=1e-5, threshold=1e-5)


    sheath.addObject('SparseGridTopology', name='sp_topology', n=gridSize,
                     fileTopology='meshes\Sheath\sheath_v5_3.obj')
    sheath.addObject('MechanicalObject', name='dofs', dx=translation[0], dy=translation[1], dz=translation[2],
                     rz=rotation[2], src='@sp_topology', restScale=restScale)

    sheath.addObject('UniformMass', totalMass=totalMass)
    sheath.addObject('MeshSpringForceField', name='Springs', noCompression=True, stiffness=stiffness, damping=damping)

    sheath.addObject('FixedConstraint', name="fixed", indices=[24, 49, 99, 74])
    sheath.addObject('UncoupledConstraintCorrection')

    visual = sheath.addChild("Visu")

    visual.addObject('MeshObjLoader', name='meshLoader_0', filename='meshes\Sheath\sheath_v5_3.obj', handleSeams=1)
    visual.addObject('OglModel', name='Visual', src='@meshLoader_0', color='yellow')
    visual.addObject('BarycentricMapping', input='@..', output='@Visual')

    triangle = sheath.addChild("TriangleSurf")

    triangle.addObject('MeshObjLoader', name='loader', filename='meshes\sheath_v5_3.obj')
    triangle.addObject('MeshTopology', src='@loader')
    triangle.addObject('MechanicalObject', src='@loader')
    triangle.addObject('SurfacePressureForceField', name='pressure_field', pressure=pressure, pulseMode=0,
                       pressureSpeed=0, mainDirection=[0, 0, 0])
    triangle.addObject('BarycentricMapping', input='@..', output='@.')

    sheath_coll = sheath.addChild('sheath_collision')

    sheath_coll.addObject('SparseGridTopology', name='sheath_loader', n=collgridSize, fileTopology='meshes\Sheath\sheath_v5_3.obj')
    sheath_coll.addObject('MechanicalObject', src='@sheath_loader')
    sheath_coll.addObject('SphereCollisionModel', simulated=1, moving=1, selfCollision=0,
                     proximity=0, radius=1.1, contactStiffness=1e4, contactFriction=0, contactRestitution=0)
    sheath_coll.addObject('BarycentricMapping', input='@..', output='@.')

    guide = sheath.addChild('guide')

    guide.addObject('EdgeSetTopologyContainer', name='guide_topology',
                    position=[0, 0, 0, 0, 0, 0.5, 0, 0, 1.0, 0, 0, 1.5, 0, 0, 2.0, 0, 0, 2.5, 0, 0, 3.0, 0, 0, 3.5, 0,
                              0, 4.0, 0, 0, 4.5, 0, 0, 5.0, 0, 0, 5.5, 0, 0,
                              6.0, 0, 0, 6.5, 0, 0, 7.0, 0, 0, 7.5, 0, 0, 8.0, 0, 0, 8.5, 0, 0, 9.0, 0, 0, 9.5, 0, 0,
                              10.0, 0, 0, 10.5, 0, 0, 11.0, 0, 0, 11.5, 0,
                              0, 12.0, 0, 0, 12.5, 0, 0, 13.0, 0, 0, 13.5, 0, 0, 14.0, 0, 0, 14.5, 0, 0, 15.0, 0, 0,
                              15.5, 0, 0, 16.0, 0, 0, 16.5, 0, 0, 17.0,
                              0, 0, 17.5, 0, 0, 18.0, 0, 0, 18.5, 0, 0, 19.0, 0, 0, 19.5, 0, 0, 20.0, 0, 0, 20.5, 0, 0,
                              21.0, 0, 0, 21.5, 0, 0, 22.0, 0, 0,
                              22.5, 0, 0, 23.0, 0, 0, 23.5, 0, 0, 24.0, 0, 0, 24.5, 0, 0, 25.0, 0, 0, 25.5, 0, 0, 26.0,
                              0, 0, 26.5, 0, 0, 27.0, 0, 0, 27.5, 0,
                              0, 28.0, 0, 0, 28.5, 0, 0, 29.0, 0, 0, 29.5, 0, 0, 30.0, 0, 0, 30.5, 0, 0, 31.0, 0, 0,
                              31.5, 0, 0, 32.0, 0, 0, 32.5, 0, 0, 33.0,
                              0, 0, 33.5, 0, 0, 34.0, 0, 0, 34.5, 0, 0, 35.0, 0, 0, 35.5, 0, 0, 36.0, 0, 0, 36.5, 0, 0,
                              37.0, 0, 0, 37.5, 0, 0, 38.0, 0, 0,
                              38.5, 0, 0, 39.0, 0, 0, 39.5, 0, 0, 40.0, 0, 0, 40.5, 0, 0, 41.0, 0, 0, 41.5, 0, 0, 42.0,
                              0, 0, 42.5, 0, 0, 43.0, 0, 0, 43.5, 0,
                              0, 44.0, 0, 0, 44.5, 0, 0, 45.0, 0, 0, 45.5, 0, 0, 46.0, 0, 0, 46.5, 0, 0, 47.0, 0, 0,
                              47.5, 0, 0, 48.0, 0, 0, 48.5, 0, 0, 49.0,
                              0, 0, 49.5, 0, 0, 50.0, 0, 0, 50.5, 0, 0, 51.0, 0, 0, 51.5, 0, 0, 52.0, 0, 0, 52.5, 0, 0,
                              53.0, 0, 0, 53.5, 0, 0, 54.0, 0, 0,
                              54.5, 0, 0, 55.0, 0, 0, 55.5, 0, 0, 56.0, 0, 0, 56.5, 0, 0, 57.0, 0, 0, 57.5, 0, 0, 58.0,
                              0, 0, 58.5, 0, 0, 59.0, 0, 0, 59.5, 0,
                              0, 60.0, 0, 0, 60.5, 0, 0, 61.0, 0, 0, 61.5, 0, 0, 62.0, 0, 0, 62.5, 0, 0, 63.0, 0, 0,
                              63.5, 0, 0, 64.0, 0, 0, 64.5, 0, 0, 65.0,
                              0, 0, 65.5, 0, 0, 66.0, 0, 0, 66.5, 0, 0, 67.0, 0, 0, 67.5, 0, 0, 68.0, 0, 0, 68.5, 0, 0,
                              69.0, 0, 0, 69.5, 0, 0, 70.0, 0, 0,
                              70.5, 0, 0, 71.0, 0, 0, 71.5, 0, 0, 72.0, 0, 0, 72.5, 0, 0, 73.0, 0, 0, 73.5, 0, 0, 74.0,
                              0, 0, 74.5, 0, 0, 75.0, 0, 0, 75.5, 0,
                              0, 76.0, 0, 0, 76.5, 0, 0, 77.0, 0, 0, 77.5, 0, 0, 78.0, 0, 0, 78.5, 0, 0, 79.0, 0, 0,
                              79.5, 0, 0, 80.0, 0, 0, 80.5, 0, 0, 81.0,
                              0, 0, 81.5, 0, 0, 82.0, 0, 0, 82.5, 0, 0, 83.0, 0, 0, 83.5, 0, 0, 84.0, 0, 0, 84.5, 0, 0,
                              85.0, 0, 0, 85.5, 0, 0, 86.0, 0, 0,
                              86.5, 0, 0, 87.0, 0, 0, 87.5, 0, 0, 88.0, 0, 0, 88.5, 0, 0, 89.0, 0, 0, 89.5, 0, 0, 90.0,
                              0, 0, 90.5, 0, 0, 91.0, 0, 0, 91.5, 0,
                              0, 92.0, 0, 0, 92.5, 0, 0, 93.0, 0, 0, 93.5, 0, 0, 94.0, 0, 0, 94.5, 0, 0, 95.0, 0, 0,
                              95.5, 0, 0, 96.0, 0, 0, 96.5, 0, 0, 97.0,
                              0, 0, 97.5, 0, 0, 98.0, 0, 0, 98.5, 0, 0, 99.0, 0, 0, 99.5, 0, 0, 100.0, 0, 0, 100.5, 0,
                              0, 101.0, 0, 0, 101.5, 0, 0, 102.0, 0,
                              0, 102.5, 0, 0, 103.0, 0, 0, 103.5, 0, 0, 104.0, 0, 0, 104.5, 0, 0, 105.0, 0, 0, 105.5, 0,
                              0, 106.0, 0, 0, 106.5, 0, 0, 107.0,
                              0, 0, 107.5, 0, 0, 108.0, 0, 0, 108.5, 0, 0, 109.0, 0, 0, 109.5, 0, 0, 110.0, 0, 0, 110.5,
                              0, 0, 111.0, 0, 0, 111.5, 0, 0,
                              112.0, 0, 0, 112.5, 0, 0, 113.0, 0, 0, 113.5, 0, 0, 114.0, 0, 0, 114.5, 0, 0, 115.0, 0, 0,
                              115.5, 0, 0, 116.0, 0, 0, 116.5, 0,
                              0, 117.0, 0, 0, 117.5, 0, 0, 118.0, 0, 0, 118.5, 0, 0, 119.0, 0, 0, 119.5, 0, 0, 120.0, 0,
                              0, 120.5, 0, 0, 121.0, 0, 0, 121.5,
                              0, 0, 122.0, 0, 0, 122.5, 0, 0, 123.0, 0, 0, 123.5, 0, 0, 124.0, 0, 0, 124.5, 0, 0, 125.0,
                              0, 0, 125.5, 0, 0, 126.0, 0, 0,
                              126.5, 0, 0, 127.0, 0, 0, 127.5, 0, 0, 128.0, 0, 0, 128.5, 0, 0, 129.0, 0, 0, 129.5, 0, 0,
                              130.0,],
                    edges=[0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14,
                           14, 15,
                           15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26,
                           27, 27,
                           28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38, 39,
                           39, 40,
                           40, 41, 41, 42, 42, 43, 43, 44, 44, 45, 45, 46, 46, 47, 47, 48, 48, 49, 49, 50, 50, 51, 51,
                           52, 52,
                           53, 53, 54, 54, 55, 55, 56, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 61, 62, 62, 63, 63, 64,
                           64, 65,
                           65, 66, 66, 67, 67, 68, 68, 69, 69, 70, 70, 71, 71, 72, 72, 73, 73, 74, 74, 75, 75, 76, 76,
                           77, 77,
                           78, 78, 79, 79, 80, 80, 81, 81, 82, 82, 83, 83, 84, 84, 85, 85, 86, 86, 87, 87, 88, 88, 89,
                           89, 90,
                           90, 91, 91, 92, 92, 93, 93, 94, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 100, 100, 101,
                           101, 102,
                           102, 103, 103, 104, 104, 105, 105, 106, 106, 107, 107, 108, 108, 109, 109, 110, 110, 111,
                           111, 112,
                           112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 120, 120, 121,
                           121, 122,
                           122, 123, 123, 124, 124, 125, 125, 126, 126, 127, 127, 128, 128, 129, 129, 130, 130, 131,
                           131, 132,
                           132, 133, 133, 134, 134, 135, 135, 136, 136, 137, 137, 138, 138, 139, 139, 140, 140, 141,
                           141, 142,
                           142, 143, 143, 144, 144, 145, 145, 146, 146, 147, 147, 148, 148, 149, 149, 150, 150, 151,
                           151, 152,
                           152, 153, 153, 154, 154, 155, 155, 156, 156, 157, 157, 158, 158, 159, 159, 160, 160, 161,
                           161, 162,
                           162, 163, 163, 164, 164, 165, 165, 166, 166, 167, 167, 168, 168, 169, 169, 170, 170, 171,
                           171, 172,
                           172, 173, 173, 174, 174, 175, 175, 176, 176, 177, 177, 178, 178, 179, 179, 180, 180, 181,
                           181, 182,
                           182, 183, 183, 184, 184, 185, 185, 186, 186, 187, 187, 188, 188, 189, 189, 190, 190, 191,
                           191, 192,
                           192, 193, 193, 194, 194, 195, 195, 196, 196, 197, 197, 198, 198, 199, 199, 200, 200, 201,
                           201, 202,
                           202, 203, 203, 204, 204, 205, 205, 206, 206, 207, 207, 208, 208, 209, 209, 210, 210, 211,
                           211, 212,
                           212, 213, 213, 214, 214, 215, 215, 216, 216, 217, 217, 218, 218, 219, 219, 220, 220, 221,
                           221, 222,
                           222, 223, 223, 224, 224, 225, 225, 226, 226, 227, 227, 228, 228, 229, 229, 230, 230, 231,
                           231, 232,
                           232, 233, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238, 239, 239, 240, 240, 241,
                           241, 242,
                           242, 243, 243, 244, 244, 245, 245, 246, 246, 247, 247, 248, 248, 249, 249, 250, 250, 251,
                           251, 252,
                           252, 253, 253, 254, 254, 255, 255, 256, 256, 257, 257, 258, 258, 259, 259, 260, 260,])

    guide.addObject('MechanicalObject', name='mobject', dx=3, dy=3, src='@guide_topology')
    guide.addObject('SphereCollisionModel', simulated=1, moving=1, selfCollision=0,
                    proximity=0, radius=0.35, contactStiffness=1e4, contactFriction=40, contactRestitution=0)
    guide.addObject('BarycentricMapping', input='@..', output='@.')

    return sheath
