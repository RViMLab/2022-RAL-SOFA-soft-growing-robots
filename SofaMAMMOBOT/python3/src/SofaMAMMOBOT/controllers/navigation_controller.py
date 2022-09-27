import Sofa
import Sofa.Core as SC
import numpy as np
import math
import pandas as pd
import os

class NavigationController(SC.Controller):

    def __init__(self, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.node = kwargs["node"]

        self.sheath = self.node.getChild('sheath')
        self.catheter = self.node.getChild('catheter')
        self.cath_base = self.catheter.getObject('cath_base')
        self.sheath_opening = self.sheath.getObject('sheath_opening')
        self.sheath_base = self.sheath.getObject('sheath_base')
        self.guide = self.sheath.getChild('guide')
        self.tendon = self.catheter.getChild('tendon')
        self.triangle = self.sheath.getChild('TriangleSurf')
        self.iteration = 0
        print('\n')
        print('\t\t----------------- MAMMOBOT SIMULATION CONTROLLER -------------------')
        print('\n')
        print('In this simulation, you can control the different objects in the scene using keyboard commands. These are the commands: ')
        print('1 - SHEATH PRESSURE FIELD:')
        print('\t * "up" increase pressure & "down" decrease pressure')
        print('2 - CATHETER MOVEMENT:')
        print('\t * Translation: "left" increase the z translation & "right" decrease the z translation')
        print('\t * Rotation: "up" increase the z rotation & "down" decrease the z rotation')
        print('3 - SHEATH MOVEMENT:')
        print('\t * Translation: "left" increase the z translation & "right" decrease the z translation')
        print('\t * Rotation: "up" increase the z rotation & "down" decrease the z rotation')
        print('4 - CATHETER TENDON REST SCALE:')
        print('\t * "up" increase restScale & "down" decrease restScale')
        print("5 - CATHETER AND SHEATH MANIPULATOR:")

        self.commands = ['1', '2', '3', '4', '5']
        self.object = '1'

        self.cath_tip_ind = [1111, 984, 855, 705, 555, 428, 299, 149]
        self.she_tip_ind = [88, 63, 38, 13]
        self.cath_base_ind = [0, 556, 985, 429]
        self.she_base_ind = [0, 1, 2, 25, 26, 27, 50, 51, 52, 75, 76, 77]


        self.mrate = 0.1
        self.cath_vrate = 0.5
        self.sheath_vrate = 0.1
        self.has_changed = False
        self.prate = -5e3
        self.changers = 210.0
        self.rsrate = 0.001
        self.change_times = 0
        self.pre_pressure = self.triangle.pressure_field.pressure.value
        self.angle = 0.0
        self.trans = 0.0
        self.sheath_vel = 0.0
        self.cath_vel = 0.0

        self.dl = 25 # mm
        self.dt_l = 50 # s
        self.dtheta = 10 # 
        self.dt_theta = 0.3 # s

        self.root = "Data"
        self.flag = False
        self.create_tendon = False


    def onKeypressedEvent(self, event):
        key = event['key']
        time = self.iteration*self.node.dt.value

        if key in self.commands:
            self.object = key

        else:
            if self.object == '1': # pressure
                # if key == '+':
                if ord(key) == 19: #up
                    self.triangle.pressure_field.pressure.value += self.prate
                    print('Pressure: ', self.triangle.pressure_field.pressure.value)
                # elif key == '-':
                if ord(key) == 21: #down
                    self.triangle.pressure_field.pressure.value -= self.prate
                    print('Pressure: ', self.triangle.pressure_field.pressure.value)

            elif self.object == '2': # cath_base
                if ord(key) == 18: #left
                    self.cath_vel += self.cath_vrate
                    with self.catheter.catheter_mobject.velocity.writeable() as vC:
                        for idx in self.cath_base_ind:
                            vC[idx] = [0, 0, self.cath_vel]
                        print('Cath base: ', vC[idx])
                elif ord(key) == 20: #right
                    self.cath_vel -= self.cath_vrate
                    with self.catheter.catheter_mobject.velocity.writeable() as vC:
                        for idx in self.cath_base_ind:
                            vC[idx] = [0, 0, self.cath_vel]
                        print('Cath base: ', vC[idx])

            elif self.object == '3': # sheath base
                if ord(key) == 18: #left
                    self.sheath_vel += self.sheath_vrate
                    with self.sheath.dofs.velocity.writeable() as vS:
                        for idx in self.she_base_ind:
                            vS[idx] = [0, 0, self.sheath_vel]
                        print('Sheath base: ', vS[idx])
                        
                elif ord(key) == 20: #right
                    self.sheath_vel -= self.sheath_vrate
                    with self.sheath.dofs.velocity.writeable() as vS:
                        for idx in self.she_base_ind:
                            vS[idx] = [0, 0, self.sheath_vel]
                        print('Seath base: ', vS[idx])

            elif self.object == '4': # cath tendon
                restScale = self.tendon.tendon_mobject.restScale.value

                if ord(key) == 19: #up
                    self.catheter.removeChild(self.tendon)
                    if restScale < 1.0:
                        createTendon(self.catheter, restScale=restScale+self.rsrate)

                if ord(key) == 21: #down
                    self.catheter.removeChild(self.tendon)
                    if restScale > 0.0:
                        createTendon(self.catheter, restScale=restScale-self.rsrate)
                self.tendon = self.catheter.getChild('tendon')
                print('RestScale: ', restScale)

            elif self.object == '5': # cath_base & sheath_base
                if ord(key) == 18: #left, move - (retreave)
                    self.cath_vel = 0.0
                    self.sheath_vel = 0.0
                    self.catheter.removeObject(self.cath_base)
                    self.sheath.removeObject(self.sheath_base)
                    self.trans = self.trans + self.dl;
                    createCathBase(self.catheter, trans=self.dl, angle=0, t=time, dt=self.dt_l)
                    createSheathBase(self.sheath, trans=2*self.dl, angle=0, t=time, dt=self.dt_l)
                    print('system translation: ' + str(self.trans) + ', end time: ' + str(time+self.dt_l) )
                elif ord(key) == 20: #right, move + (insert)
                    self.cath_vel = 0.0
                    self.sheath_vel = 0.0
                    self.catheter.removeObject(self.cath_base)
                    self.sheath.removeObject(self.sheath_base)
                    self.trans = self.trans - self.dl;
                    createCathBase(self.catheter, trans=-self.dl, angle=0, t=time, dt=self.dt_l)
                    createSheathBase(self.sheath, trans=-2*self.dl, angle=0, t=time, dt=self.dt_l)
                    print('system translation: ' + str(self.trans) + ', end time: ' + str(time+self.dt_l) )
                if ord(key) == 19: #up, rotate -
                    self.cath_vel = 0.0
                    self.sheath_vel = 0.0
                    self.catheter.removeObject(self.cath_base)
                    self.sheath.removeObject(self.sheath_base)
                    self.angle = self.angle + self.dtheta;
                    createCathBase(self.catheter, trans=0, angle=self.dtheta*math.pi/180, t=time, dt=self.dt_theta)
                    createSheathBase(self.sheath, trans=0, angle=self.dtheta*math.pi/180, t=time, dt=self.dt_theta)
                    self.sheath.removeObject(self.sheath_opening)
                    createSheathOpening(self.sheath, trans=0, angle=self.dtheta*math.pi/180, t=time, dt=self.dt_theta)
                    print('system translation: ' + str(self.angle) + ', end time: ' + str(time+self.dt_theta) )
                elif ord(key) == 21: #down, rotate +
                    self.cath_vel = 0.0
                    self.sheath_vel = 0.0
                    self.catheter.removeObject(self.cath_base)
                    self.sheath.removeObject(self.sheath_base)
                    self.angle = self.angle - self.dtheta;
                    createCathBase(self.catheter, trans=0, angle=-self.dtheta*math.pi/180, t=time, dt=self.dt_theta)
                    createSheathBase(self.sheath, trans=0, angle=-self.dtheta*math.pi/180, t=time, dt=self.dt_theta)
                    self.sheath.removeObject(self.sheath_opening)
                    createSheathOpening(self.sheath, trans=0, angle=-self.dtheta*math.pi/180, t=time, dt=self.dt_theta)
                    print('system translation: ' + str(self.angle) + ', end time: ' + str(time+self.dt_theta) )
                elif key == '=': # tendon release/push
                    restScale = self.tendon.tendon_mobject.restScale.value
                    self.catheter.removeChild(self.tendon)
                    createTendon(self.catheter, restScale=restScale+self.rsrate)
                    self.tendon = self.catheter.getChild('tendon')
                    print('RestScale: ', restScale)
                elif key == '-': # tendon pull
                    restScale = self.tendon.tendon_mobject.restScale.value
                    self.catheter.removeChild(self.tendon)
                    createTendon(self.catheter, restScale=restScale-self.rsrate)
                    self.tendon = self.catheter.getChild('tendon')
                    print('RestScale: ', restScale)
                elif key == '0': # stop
                    self.cath_vel = 0.0
                    self.sheath_vel = 0.0
                    self.catheter.removeObject(self.cath_base)
                    self.sheath.removeObject(self.sheath_base)
                    createCathBase(self.catheter, trans=0, angle=0, t=time, dt=self.dt_theta)
                    createSheathBase(self.sheath, trans=0, angle=0, t=time, dt=self.dt_theta)
                    self.sheath.removeObject(self.sheath_opening)
                    createSheathOpening(self.sheath, trans=0, angle=0, t=time, dt=self.dt_theta)
                    print('Motion stop!')
        return


    def onAnimateEndEvent(self, event):

        time = self.iteration*self.node.dt.value

        if time == self.node.dt.value:            
            print('wait for transient dynamic to damp...')

        if time == 150 * self.node.dt.value:
            with self.sheath.dofs.velocity.writeable() as vS:
                for idx in self.she_base_ind:
                    vS[idx] = [0, 0, 0]
            print('You may proceed now...')

        return
    

def createTendon(parentNode, indices=[ 429, 430, 430, 431, 431, 432, 432, 433, 433, 434, 434, 435, 435, 436, 436, 437, 437, 438, 438, 439, 439, 440, 440, 441, 441, 442, 442, 443, 443, 444, 444, 445, 445, 446, 446, 447, 447, 448, 448, 449, 449, 450, 450, 451, 451, 452, 452, 453, 453, 454, 454, 455, 455, 456, 456, 457, 457, 458, 458, 459, 459, 460, 460, 461, 461, 462, 462, 463, 463, 464, 464, 465, 465, 466, 466, 467, 467, 468, 468, 469, 469, 470, 470, 471, 471, 472, 472, 473, 473, 474, 474, 475, 475, 476, 476, 477, 477, 478, 478, 479, 479, 480, 480, 481, 481, 482, 482, 483, 483, 484, 484, 485, 485, 486, 486, 487, 487, 488, 488, 489, 489, 490, 490, 491, 491, 492, 492, 493, 493, 494, 494, 495, 495, 496, 496, 497, 497, 498, 498, 499, 499, 500, 500, 501, 501, 502, 502, 503, 503, 504, 504, 505, 505, 506, 506, 507, 507, 508, 508, 509, 509, 510, 510, 511, 511, 512, 512, 513, 513, 514, 514, 515, 515, 516, 516, 517, 517, 518, 518, 519, 519, 520, 520, 521, 521, 522, 522, 523, 523, 524, 524, 525, 525, 526, 526, 527, 527, 528, 528, 529, 529, 530, 530, 531, 531, 532, 532, 533, 533, 534, 534, 535, 535, 536, 536, 537, 537, 538, 538, 539, 539, 540, 540, 541, 541, 542, 542, 543, 543, 544, 544, 545, 545, 546, 546, 547, 547, 548, 548, 549, 549, 550, 550, 551, 551, 552, 552, 553, 553, 554, 554, 555, 985, 986, 986, 987, 987, 988, 988, 989, 989, 990, 990, 991, 991, 992, 992, 993, 993, 994, 994, 995, 995, 996, 996, 997, 997, 998, 998, 999, 999, 1000, 1000, 1001, 1001, 1002, 1002, 1003, 1003, 1004, 1004, 1005, 1005, 1006, 1006, 1007, 1007, 1008, 1008, 1009, 1009, 1010, 1010, 1011, 1011, 1012, 1012, 1013, 1013, 1014, 1014, 1015, 1015, 1016, 1016, 1017, 1017, 1018, 1018, 1019, 1019, 1020, 1020, 1021, 1021, 1022, 1022, 1023, 1023, 1024, 1024, 1025, 1025, 1026, 1026, 1027, 1027, 1028, 1028, 1029, 1029, 1030, 1030, 1031, 1031, 1032, 1032, 1033, 1033, 1034, 1034, 1035, 1035, 1036, 1036, 1037, 1037, 1038, 1038, 1039, 1039, 1040, 1040, 1041, 1041, 1042, 1042, 1043, 1043, 1044, 1044, 1045, 1045, 1046, 1046, 1047, 1047, 1048, 1048, 1049, 1049, 1050, 1050, 1051, 1051, 1052, 1052, 1053, 1053, 1054, 1054, 1055, 1055, 1056, 1056, 1057, 1057, 1058, 1058, 1059, 1059, 1060, 1060, 1061, 1061, 1062, 1062, 1063, 1063, 1064, 1064, 1065, 1065, 1066, 1066, 1067, 1067, 1068, 1068, 1069, 1069, 1070, 1070, 1071, 1071, 1072, 1072, 1073, 1073, 1074, 1074, 1075, 1075, 1076, 1076, 1077, 1077, 1078, 1078, 1079, 1079, 1080, 1080, 1081, 1081, 1082, 1082, 1083, 1083, 1084, 1084, 1085, 1085, 1086, 1086, 1087, 1087, 1088, 1088, 1089, 1089, 1090, 1090, 1091, 1091, 1092, 1092, 1093, 1093, 1094, 1094, 1095, 1095, 1096, 1096, 1097, 1097, 1098, 1098, 1099, 1099, 1100, 1100, 1101, 1101, 1102, 1102, 1103, 1103, 1104, 1104, 1105, 1105, 1106, 1106, 1107, 1107, 1108, 1108, 1109, 1109, 1110, 1110, 1111], restScale=1.0):
    tendon = parentNode.addChild('tendon')
    tendon.addObject('EdgeSetTopologyContainer', name='tendon_container', position='@../sp_topology.position',
                     edges=indices)
    tendon.addObject('MechanicalObject', name='tendon_mobject', topology='@tendon_container', restScale=restScale)
    tendon.addObject('UniformMass', totalMass=3.14e-4, topology='@tendon_container')
    tendon.addObject('MeshSpringForceField', name='tendon_springs', topology='@tendon_container', drawMode=4,
                     template='Vec3d', linesStiffness=2e9, linesDamping=40, drawSpringSize=20, noCompression=True)
    tendon.addObject('IdentityMapping', input='@..', output='@.')
    tendon.init()

    
def createCathBase(parentNode, trans=0.0, angle=0.0, t=0.0, dt=0.1):
    cath_base = parentNode.addObject('AffineMovementConstraint', name='cath_base', template='Vec3d', indices=[0, 556, 985, 429],
                      beginConstraintTime=t, endConstraintTime=t+dt, translation=[0, 0, trans],
                      rotation=[[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    # parentNode.init()
    cath_base.init()
    

def createSheathBase(parentNode, trans=0.0, angle=0.0, t=0.0, dt=0.1):
    sheath_base = parentNode.addObject('AffineMovementConstraint', name='sheath_base', template='Vec3d', indices=[0, 25, 50, 75],
                      beginConstraintTime=t, endConstraintTime=t+dt, translation=[0, 0, trans],
                      rotation=[[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    # parentNode.init()
    sheath_base.init()
    

def createSheathOpening(parentNode, trans=0.0, angle=0.0, t=0.0, dt=0.1):
    sheath_opening = parentNode.addObject('AffineMovementConstraint', name='sheath_opening', template='Vec3d', indices=[24, 49, 99, 74],
                      beginConstraintTime=t, endConstraintTime=t+dt, translation=[0, 0, trans],
                      rotation=[[math.cos(angle), -math.sin(angle), 0], [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    # parentNode.init()
    sheath_opening.init()