import Sofa
import Sofa.Core as SC

def CylinderNode(parent, name="Cylinder", gridSize=[5, 15, 5], axis=[0, 1, 0], length=30.0, radius=5.0, translation=[0, 0, 0]):

    cylinder = rootNode.addChild('cylinder1')
    cylinder.addObject('CylinderGridTopology', name='cyl_top', nx=gridSize[0], ny=gridSize[1], nz=gridSize[2],
    length=length, radius=radius, axis=axis)
    cylinder.addObject('MechanicalObject', dz=translation[2], dy=translation[1], dx=translation[0])
    cylinder.addObject('TriangleCollisionModel', group=2, simulated=0)
    cylinder.addObject('LineCollisionModel', group=2, simulated=0)
    cylinder.addObject('PointCollisionModel', group=2, simulated=0)

    cyl_visual = cylinder.addChild('Visual')
    cyl_visual.addObject('OglModel', src="@../cyl_top", color="white")
    cyl_visual.addObject('IdentityMapping')
