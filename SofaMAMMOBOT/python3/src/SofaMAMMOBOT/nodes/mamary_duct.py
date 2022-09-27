import Sofa
import Sofa.Core as SC

def MammaryDuct(parent, translation=[-1350, 19230, 2550], rotation=[-180, 0, 0], scale: float = 15.0):

    duct = rootNode.addChild('mammary_duct')
    duct.addObject('MeshSTLLoader', name='loader',
                   filename="meshes/Phantoms/Breast_Lobules_x1.stl")
    duct.addObject('MeshTopology', src="@loader")
    duct.addObject('MechanicalObject', src="@loader", dx=translation[0], dz=translation[2], dy=translation[1], rx=rotation[0],
                    ry=rotation[1], rz=rotation[2], scale=scale)
    duct.addObject('TriangleCollisionModel', simulated=0, moving=0, selfCollision=0,
                   proximity=0, contactStiffness=1e4, contactFriction=40)
    duct.addObject('LineCollisionModel', simulated=0, moving=0, selfCollision=0,
                   proximity=0, contactStiffness=1e4, contactFriction=40)
    duct.addObject('PointCollisionModel', simulated=0, moving=0, selfCollision=0,
                   proximity=0, contactStiffness=1e4, contactFriction=40)

    visu_duct = duct.addChild('Visual')
    visu_duct.addObject('MeshSTLLoader', name='loader',
                   filename="meshes/Breast_Lobules_x1.stl")
    visu_duct.addObject('OglModel', src="@loader", name='Visu')
    visu_duct.addObject('IdentityMapping', input='@..', output='@Visu')