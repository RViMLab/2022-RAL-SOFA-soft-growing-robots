import Sofa
import Sofa.Core as SC
import numpy as np
import pandas as pd
import os

class SheathController(SC.Controller):

    def __init__(self, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.node = kwargs["node"]

        self.sheath = self.node.getChild('Sheath')
        self.guide = self.sheath.getChild('guide')
        #self.tendon = self.catheter.getChild('tendon')
        self.triangle = self.sheath.getChild('TriangleSurf')
        self.iteration = 0
        print('\n')
        print('\t\t----------------- MAMMOBOT SIMULATION CONTROLLER -------------------')
        print('\n')
        print('In this simulation, you can control the different objects in the scene using keyboard commands. These are the commands: ')
        print('1 - SHEATH PRESSURE FIELD: "+" increase pressure & "-" decrease pressure')
        print('2 - SHEATH MOVEMENT: Using the arrows')
        print('\t * Translation: "left" increase the z translation & "right" decrease the z translation')
        print('\t * Rotation: "up" increase the z rotation & "down" decrease the z rotation')

        self.commands = ['1', '2', '3']
        self.object = '1'

        self.she_tip = [26, 71, 116, 161]
        self.moving = [0, 45, 90, 135]

        self.mrate = 0.1
        self.sheath_vrate = 2.877
        self.has_changed = False
        self.prate = -1e4
        self.change_times = 0
        self.pre_pressure = self.triangle.pressure_field.pressure.value

        os.makedirs("results/sheath_grow", exist_ok=True)
        self.root = "results/sheath_grow"

        print('Files will be saved: ', self.root)
        self.flag = False
        self.create_tendon = False
        self.load = 0.0

    def onKeypressedEvent(self, event):
        key = event['key']

        if key in self.commands:
            self.object = key

        else:
            if self.object == '1':
                if key == '+':
                    self.triangle.pressure_field.pressure.value += self.prate
                    print('Pressure: ', self.triangle.pressure_field.pressure.value)
                    self.flag = True

                elif key == '-':
                    self.triangle.pressure_field.pressure.value -= self.prate
                    print('Pressure: ', self.triangle.pressure_field.pressure.value)
                    self.flag = True

            elif self.object == '2':
                if ord(key) == 18: #left
                    with self.sheath.dofs.velocity.writeable() as vS:
                        for idx in self.moving:
                            vS[idx] += [0, 0, self.sheath_vrate]
                        print('Sheath base: ', vS[idx])
                    #with self.guide.mobject.velocity.writeable() as vG:
                    #    vG[self.max_ind] -= [0, 0, self.cath_vrate]

                elif ord(key) == 20: #right
                    with self.sheath.dofs.velocity.writeable() as vS:
                        for idx in self.moving:
                            vS[idx] -= [0, 0, self.sheath_vrate]
                        print('Seath base: ', vS[idx])
                    #with self.guide.mobject.velocity.writeable() as vG:
                    #    vG[self.max_ind] -= [0, 0, self.cath_vrate]

        return

    def onAnimateBeginEvent(self, event):

        self.iteration += 1

        with self.sheath.dofs.position.writeable() as pS:
            self.max_ind = pS.argmax(axis=0)[2]
            max_pos = pS[self.max_ind][2]

        if max_pos > self.changers and not self.has_changed and self.iteration > 1000:
            print(self.max_ind)
            self.has_changed = True
            self.create_tendon = True

            with self.sheath.dofs.position.writeable() as pS:
                she_fixed = []
                for i, pos in enumerate(pS):
                    if pos[2] < 280.0:
                        she_fixed.append(i)

            self.sheath.fixed.indices.value = she_fixed

            with self.sheath.dofs.velocity.writeable() as vS:
                for idx in self.sheath.fixed.indices.value:
                    vS[idx] = [0, 0, 0]


            self.she_x, self.she_y, self.she_z = [], [], []

            self.triangle.pressure_field.pressure.value = -1e4
            self.pre_pressure = self.triangle.pressure_field.pressure.value
            createForceField(self.sheath, indices=[72])
            self.force = self.sheath.ForceField
            print('RestScale updated! Motion and pressure field stopped at ', self.iteration)

        if self.has_changed:
            with self.sheath.dofs.position.writeable() as pS:
                self.she_x.append(np.mean([pS[117][0], pS[72][0], pS[27][0], pS[162][0]]))
                self.she_y.append(np.mean([pS[117][1], pS[72][1], pS[27][1], pS[162][1]]))
                self.she_z.append(np.mean([pS[117][2], pS[72][2], pS[27][2], pS[162][2]]))

        return

    def onAnimateEndEvent(self, event):

        if self.flag:
            final_vec = np.concatenate([np.reshape(self.she_x, (len(self.she_x), 1)),
                                        np.reshape(self.she_y, (len(self.she_y), 1)),
                                        np.reshape(self.she_z, (len(self.she_z), 1))], axis=1)

            pos_df = pd.DataFrame(final_vec, columns=['SheX', 'SheY', 'SheZ'])
            path = self.root + '/' + str(int(self.pre_pressure)) + 'kPa.csv'
            pos_df.to_csv(path_or_buf=path, sep=',')
            self.she_x, self.she_y, self.she_z = [], [], []
            print('Saved file: ', path)
            self.flag = False
            self.pre_pressure = self.triangle.pressure_field.pressure.value

        return

def createForceField(parentNode, indices, initialF=0.0):

    parentNode.addObject('ConstantForceField', name='ForceField', indices=indices, forces=[0, initialF, 0])
    parentNode.ForceField.init()