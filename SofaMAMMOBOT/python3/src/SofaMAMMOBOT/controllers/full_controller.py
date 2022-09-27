import Sofa
import Sofa.Core as SC
import numpy as np
import pandas as pd
import os

class SheathCatheterController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.node = kwargs["node"]

        self.sheath = self.node.getChild('Sheath')
        self.catheter = self.node.getChild('catheter')
        self.guide = self.sheath.getChild('guide')
        #self.tendon = self.catheter.getChild('tendon')
        self.triangle = self.sheath.getChild('TriangleSurf')
        self.iteration = 0
        print('\n')
        print('\t\t----------------- MAMMOBOT SIMULATION CONTROLLER -------------------')
        print('\n')
        print('In this simulation, you can control the different objects in the scene using keyboard commands. These are the commands: ')
        print('1 - SHEATH PRESSURE FIELD: "+" increase pressure & "-" decrease pressure')
        print('2 - CATHETER MOVEMENT: Using the arrows')
        print('\t * Translation: "left" increase the z translation & "right" decrease the z translation')
        print('\t * Rotation: "up" increase the z rotation & "down" decrease the z rotation')
        print('3 - SHEATH MOVEMENT: Using the arrows')
        print('\t * Translation: "left" increase the z translation & "right" decrease the z translation')
        print('\t * Rotation: "up" increase the z rotation & "down" decrease the z rotation')
        print('4 - CATHETER TENDON REST SCALE: "+" increase restScale & "-" decrease restScale')
        print('5 - CATHETER EXTERNAL FX FORCE: "+" increase Fx & "-" decrease Fx')


        self.commands = ['1', '2', '3', '4', '5', '6', '7']
        self.object = '1'

        self.cath_tip = [1111, 984, 855, 705, 555, 428, 299, 149]
        self.she_tip = [88, 63, 38, 13]
        self.cath_base = [0, 1, 791, 792, 1038, 1039, 1829, 1830]
        self.she_base = [0, 1, 2, 3, 4, 5, 6, 7, 8, 45, 46, 47, 48, 49, 50, 51, 52, 53, 90, 91, 92, 93, 94, 95, 96, 97, 98, 135, 136, 137, 138, 139, 140, 141, 142, 143]

        self.mrate = 0.1
        self.cath_vrate = 0.5
        self.sheath_vrate = 0.1
        self.has_changed = False
        self.prate = -1e4
        self.changers = 360.0
        self.rsrate = 0.001
        self.change_times = 0
        self.pre_pressure = self.triangle.pressure_field.pressure.value
        self.cath_surface = self.catheter.getChild('cath_surface')
        self.fscale = 1.0
        self.load = 0
        self.rs_max = False

        os.makedirs("results/fullsystem", exist_ok=True)
        self.root = "results/fullsystem"
        print('Files will be saved at: ', self.root)

        self.flag = False
        self.create_tendon = False
        self.create_force = False
        self.falling = False
        self.cath_stop = False

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
                    with self.catheter.catheter_mobject.velocity.writeable() as vC:
                        for idx in self.cath_base:
                            vC[idx] += [0, 0, self.cath_vrate]
                        print('Cath base: ', vC[idx])
                elif ord(key) == 20: #right
                    with self.catheter.catheter_mobject.velocity.writeable() as vC:
                        for idx in self.cath_base:
                            vC[idx] -= [0, 0, self.cath_vrate]
                        print('Cath base: ', vC[idx])

            elif self.object == '3':
                if ord(key) == 18: #left
                    with self.sheath.dofs.velocity.writeable() as vS:
                        for idx in self.she_base:
                            vS[idx] += [0, 0, self.sheath_vrate]
                        print('Sheath base: ', vS[idx])
                    #with self.guide.mobject.velocity.writeable() as vG:
                    #    vG[self.max_ind] -= [0, 0, self.cath_vrate]

                elif ord(key) == 20: #right
                    with self.sheath.dofs.velocity.writeable() as vS:
                        for idx in self.she_base:
                            vS[idx] -= [0, 0, self.sheath_vrate]
                        print('Seath base: ', vS[idx])
                    #with self.guide.mobject.velocity.writeable() as vG:
                    #    vG[self.max_ind] -= [0, 0, self.cath_vrate]

            elif self.object == '4':
                restScale = self.tendon.tendon_mobject.restScale.value
                if key == '+':
                    self.catheter.removeChild(self.tendon)
                    if restScale < 1.0:
                        createTendon(self.catheter, restScale=restScale+self.rsrate)
                elif key == '-':
                    self.catheter.removeChild(self.tendon)
                    if restScale > 0.0:
                        createTendon(self.catheter, restScale=restScale-self.rsrate)
                self.tendon = self.catheter.getChild('tendon')
                print('RestScale: ', self.tendon.tendon_mobject.restScale.value)

            elif self.object == '5':
                if key == '+':
                    self.load += 1.0
                    val_force = self.load * -9.81e3
                    with self.force.forces.writeable() as fC:
                        fC[0][1] = -val_force
                elif key == '-':
                    self.load -= 1.0
                    val_force = self.load * -9.81e3
                    with self.force.forces.writeable() as fC:
                        fC[0][1] = val_force
                print('Catheter tip force: ', self.force.forces.value)

        return

    def onAnimateBeginEvent(self, event):

        self.iteration += 1

        with self.guide.mobject.position.writeable() as pS:
            self.max_ind = pS.argmax(axis=0)[2]
            max_pos = pS[self.max_ind][2]

        with self.catheter.catheter_mobject.position.writeable() as pC:
            max_ind = pC.argmax(axis=0)[2]
            cath_max = pC[max_ind][2]

        if cath_max > 355 and not self.cath_stop:
            print('Catheter stopped', self.iteration)
            self.cath_stop = True
            with self.catheter.catheter_mobject.position.writeable() as pC:
                cat_fixed_pos = []
                for a, pos in enumerate(pC):
                    if pos[2] < 150.0:
                        cat_fixed_pos.append(a)
                self.pre_pos = abs(np.asarray([pC[790][1], pC[539][1], pC[269][1]]).mean())


            with self.catheter.catheter_mobject.velocity.writeable() as vC:
                for idx in cat_fixed_pos:
                    vC[idx] = [0, 0, 0]
            self.catheter.FixedConstraint.indices.value = cat_fixed_pos


        if max_pos > self.changers and not self.has_changed:
            self.has_changed = True
            self.create_tendon = True
            self.create_force = True

            with self.sheath.dofs.position.writeable() as pS:
                she_fixed_pos = []
                print(pS.argmax(axis=0)[2])
                for i, pos in enumerate(pS):
                    if pos[2] < 280.0:
                        she_fixed_pos.append(i)

            with self.catheter.catheter_mobject.position.writeable() as pC:
                cat_fixed_pos = []
                for a, pos in enumerate(pC):
                    if pos[2] < 150.0:
                        cat_fixed_pos.append(a)

                self.pre_pos = abs(np.asarray([pC[790][1], pC[539][1], pC[269][1]]).mean())


            self.sheath.fixed.indices.value = she_fixed_pos
            self.catheter.FixedConstraint.indices.value = cat_fixed_pos

            with self.catheter.catheter_mobject.velocity.writeable() as vC:
                for idx in cat_fixed_pos:
                    vC[idx] = [0, 0, 0]

            with self.sheath.dofs.velocity.writeable() as vS:
                for idx in she_fixed_pos:
                    vS[idx] = [0, 0, 0]

            self.cat_x, self.cat_y, self.cat_z = [], [], []
            self.she_x, self.she_y, self.she_z = [], [], []
            self.count = 0

            self.triangle.pressure_field.pressure.value = -1e4
            print('RestScale updated! Motion and pressure field stopped at ', self.iteration)




        if self.has_changed:

            with self.catheter.catheter_mobject.position.writeable() as pC:
                self.cat_x.append(np.asarray([pC[790][0], pC[539][0], pC[269][0]]).mean())
                self.cat_y.append(np.asarray([pC[790][1], pC[539][1], pC[269][1]]).mean())
                diff = self.pre_pos - abs(np.asarray([pC[790][1], pC[539][1], pC[269][1]]).mean())
                self.pre_pos = abs(np.asarray([pC[790][1], pC[539][1], pC[269][1]]).mean())
                if diff > 0 and not self.falling and self.pre_pos > 15:
                    self.count += 1
                    if self.count == 10:
                        self.flag = True
                elif (diff < 0) and self.falling and self.pre_pos < 15:
                    self.falling = False
                self.cat_z.append(np.asarray([pC[790][2], pC[539][2], pC[269][2]]).mean())

            with self.sheath.dofs.position.writeable() as pS:
                self.she_x.append(np.asarray([pS[162][0], pS[117][0], pS[72][0], pS[27][0]]).mean())
                self.she_y.append(np.asarray([pS[162][1], pS[117][1], pS[72][1], pS[27][1]]).mean())
                self.she_z.append(np.asarray([pS[162][2], pS[117][2], pS[72][2], pS[27][2]]).mean())

        return

    def onAnimateEndEvent(self, event):

        time = self.iteration*self.node.dt.value

        if self.iteration % 1000 == 0:
            print('Iteration: ', self.iteration)

        if time == 0.13:
            self.sheath.fixed.indices.value = [44, 89, 179, 134, 0, 1, 2, 3, 4, 5, 6, 7, 8, 45, 46, 47, 48, 49, 50, 51, 52, 53, 90, 91, 92, 93, 94, 95, 96, 97, 98, 135, 136, 137, 138, 139, 140, 141, 142, 143]
            with self.sheath.dofs.velocity.writeable() as vS:
                for idx in self.she_base:
                    vS[idx] = [0, 0, 0]
                for idx in self.she_base:
                    vS[idx] = [0, 0, 1.]

        if time == 2.0:
            with self.catheter.catheter_mobject.velocity.writeable() as vC:
                for idx in self.cath_base:
                    vC[idx] = [0, 0, 0.5]

        if self.flag:
            self.triangle.pressure_field.pressure.value += self.prate

            final_vec = np.concatenate([np.reshape(self.cat_x, (len(self.cat_x), 1)),
                                        np.reshape(self.cat_y, (len(self.cat_y), 1)),
                                        np.reshape(self.cat_z, (len(self.cat_z), 1)),
                                        np.reshape(self.she_x, (len(self.she_x), 1)),
                                        np.reshape(self.she_y, (len(self.she_y), 1)),
                                        np.reshape(self.she_z, (len(self.she_z), 1))], axis=1)


            pos_df = pd.DataFrame(final_vec, columns=['Cat X', 'Cat Y', 'CatZ', 'SheX', 'SheY', 'SheZ'])
            path = self.root + '/' + str(int(self.pre_pressure)) + 'kPa.csv'
            pos_df.to_csv(path_or_buf=path, sep=',')
            self.cat_x, self.cat_y, self.cat_z = [], [], []
            self.she_x, self.she_y, self.she_z = [], [], []
            print('Saved file: ', path)
            self.flag = False
            self.pre_pressure = self.triangle.pressure_field.pressure.value
            self.falling = True
            self.count = 0


        if self.create_tendon:
            createTendon(self.catheter, restScale=0.901)
            self.tendon = self.catheter.getChild('tendon')
            self.create_tendon = False
            print('Tendon created!')

        if self.create_force:
            createForceField(self.catheter, [790], initialF=5*3.5*9.81e4)
            self.force = self.catheter.ForceField
            self.create_force = False
            self.first_fall = True
            print('Force field created!')

        return

def createTendon(parentNode, restScale=1.0, indices=[i for i in range(2003, 2075)]):

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

    parentNode.addObject('ConstantForceField', name='ForceField', indices=indices, forces=[0, initialF, 0])
    parentNode.ForceField.init()