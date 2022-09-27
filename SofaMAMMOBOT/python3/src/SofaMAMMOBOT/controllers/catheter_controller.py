import Sofa
import Sofa.Core as SC
import os
import numpy as np
import pandas as pd

class CatheterController(SC.Controller):

    def __init__(self, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.node = kwargs["node"]

        self.catheter = self.node.getChild('catheter')
        self.tendon = self.catheter.getChild('tendon')
        self.iteration = 0
        print('\n')
        print('\t\t----------------- CATHETER LOAD SIMULATION CONTROLLER -------------------')
        print('\n')
        print('In this simulation, you can control the different objects in the scene using keyboard commands. These are the commands: ')
        print('1 - CATHETER REST SCALE: "+" increase internal tension & "-" decrease internal tension')
        print('2 - CATHETER TIP LOAD: "+" increase the load & "-" decrease the tip load')


        self.commands = ['1', '2',]
        self.object = '1'

        self.cath_tip = [790]

        self.rsrate = 0.001
        self.load = 0

        os.makedirs("results/catheter", exist_ok=True)
        self.root = "results/catheter"

        self.create_force = True
        self.rschange = False
        self.cat_x, self.cat_y, self.cat_z = [], [], []
        createForceField(self.catheter, self.cath_tip, initialF=0)
        self.force = self.catheter.ForceField
        self.flag = False

    def onKeypressedEvent(self, event):
        key = event['key']

        if key in self.commands:
            self.object = key

        else:
            if self.object == '1':
                restScale = self.tendon.tendon_mobject.restScale.value
                if key == '+':
                    self.catheter.removeChild(self.tendon)
                    if restScale < 1.0:
                        createTendon(self.catheter, restScale=restScale+self.rsrate)
                        self.flag = True
                elif key == '-':
                    self.catheter.removeChild(self.tendon)
                    if restScale > 0.0:
                        createTendon(self.catheter, restScale=restScale-self.rsrate)
                        self.flag = True
                self.tendon = self.catheter.getChild('tendon')
                print('RestScale: ', self.tendon.tendon_mobject.restScale.value)

            elif self.object == '2':
                if key == '+':
                    self.load += 1
                    force = self.load * -9.81e3
                    with self.force.forces.writeable() as fC:
                        fC[0][2] = -force
                elif key == '-':
                    self.load -= 1
                    force = self.load * -9.81
                    with self.force.force.writeable() as fC:
                        fC[1] = force
                print('Catheter tip force: ', self.force.force.value)
        return

    def onAnimateBeginEvent(self, event):

        self.iteration += 1

        with self.catheter.catheter_mobject.position.writeable() as pC:
            self.cat_x.append(np.asarray([pC[1111][0], pC[984][0], pC[855][0], pC[705][0],
                                            pC[555][0], pC[428][0], pC[299][0], pC[149][0]]).mean())
            self.cat_y.append(np.asarray([pC[1111][1], pC[984][1], pC[855][1], pC[705][1],
                                            pC[555][1], pC[428][1], pC[299][1], pC[149][1]]).mean())
            self.cat_z.append(np.asarray([pC[1111][2], pC[984][2], pC[855][2], pC[705][2],
                                            pC[555][2], pC[428][2], pC[299][2], pC[149][2]]).mean())
        return

    def onAnimateEndEvent(self, event):

        if self.flag:
            final_vec = np.concatenate([np.reshape(self.cat_x, (len(self.cat_x), 1)),
                                        np.reshape(self.cat_y, (len(self.cat_y), 1)),
                                        np.reshape(self.cat_z, (len(self.cat_z), 1))], axis=1)

            pos_df = pd.DataFrame(final_vec, columns=['Cat X', 'Cat Y', 'CatZ'])
            path = self.root + '/' + str(self.tendon.tendon_mobject.restScale.value) + 'rs.csv'
            pos_df.to_csv(path_or_buf=path, sep=',')
            self.cat_x, self.cat_y, self.cat_z = [], [], []
            print('Saved file: ', path)
            self.flag = False

        return

def createTendon(parentNode, indices=[i for i in range(2003, 2075)], restScale=1.0):

    tendon = parentNode.addChild('tendon')

    tendon.addObject('EdgeSetTopologyContainer', name='tendon_container', position='@../sp_topology.position',
                     edges=indices)
    tendon.addObject('MechanicalObject', name='tendon_mobject', topology='@tendon_container', restScale=restScale)
    tendon.addObject('UniformMass', totalMass=3.14e-4, topology='@tendon_container')
    tendon.addObject('MeshSpringForceField', name='tendon_springs', topology='@tendon_container', drawMode=4,
                     template='Vec3d', linesStiffness=2e9, linesDamping=40, drawSpringSize=20, noCompression=True)

    tendon.addObject('IdentityMapping', input='@..', output='@.')

    tendon.init()

def createForceField(parentNode, indices, initialF=0.0):

    parentNode.addObject('ConstantForceField', name='ForceField', indices=indices, forces=[0, initialF, 0], showArrowSize=0.001)
    parentNode.ForceField.init()
