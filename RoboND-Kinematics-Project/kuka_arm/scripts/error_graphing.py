from mpmath import *
from sympy import *
from sympy import pi
import matplotlib as mpl
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pickle
from time import strftime

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

class kuka_path(object):

    def __init__(self):
        self.time = strftime("%Y-%m-%d %H:%M:%S")
        
        self.Px = []
        self.Py = []
        self.Pz = []
        
        self.poses = 0
        
        self.error = []
        
        self.theta1 = []
        self.theta2 = []
        self.theta3 = []
        self.theta3DH = []
        self.theta4 = [] 
        self.theta5 = [] 
        self.theta6 = []
        
        
    def test_data(self):
        self.Px = [2.0899277938636036, 2.009752633592428, 1.9000848965067405, 1.7617274351710994, 1.5959231768337183, 1.434430729928176, 1.2555388814505637, 1.0783716308614557, 0.8892321802099534, 0.7285812449384668, 0.561843239082426, 0.41052640866381823, 0.25577081173405686, 0.14088175226379707, 0.024764307596414792, -0.0923141267863839, -0.21008030586064055]
        self.Py = [0.900103192565269, 1.134512765263691, 1.3612129262225285, 1.5765438149612214, 1.776935009449351, 1.9331319804170062, 2.0740016586342604, 2.1875576035013826, 2.2854271143090847, 2.352211369403496, 2.407350130977553, 2.4458614779037915, 2.4745856905761854, 2.4892134221892337, 2.4983650960979182, 2.501956399444331, 2.499916118786028]
        self.Pz = [1.580880285494445, 1.5959549051174429, 1.6082425798005922, 1.6177021323688294, 1.6243427263967332, 1.6278139707163635, 1.6293493611994394, 1.62913345654739, 1.6274599214957721, 1.625106195957255, 1.6219195409439349, 1.6184686803273, 1.6144626465092982, 1.6112113651337805, 1.607707585974449, 1.6039703089940156, 1.6000188571379925]
        
        self.poses = 17
        
        self.error = [4.57756679852224e-16, 3.14018491736755e-16, 2.22044604925031e-16, 3.14018491736755e-16, 4.96506830649455e-16, 3.14018491736755e-16, 5.43895982204207e-16, 4.96506830649455e-16, 3.14018491736755e-16, 0, 9.15513359704447e-16, 2.22044604925031e-16, 5.23691153334427e-16, 1.94289029309402e-16, 4.54590236402796e-16, 4.44089209850063e-16, 4.84730289145668e-16]
        
        self.theta1 = [0.466701126161434, 0.585832828163094, 0.704515817798144, 0.822690850928407, 0.940311385904693, 1.03995014750744, 1.13914923499578, 1.22927774779088, 1.31903291689529, 1.39130254803090, 1.46333347476573, 1.52662357979521, -1.55185657117656 + pi, -1.50584644679682 + pi, -1.45992623641283 + pi, -1.41409361239856 + pi, -1.36834606285451 + pi]
        self.theta2 = [-1.40612271784094 + pi/2, -1.36973739022934 + pi/2, -1.33264580430497 + pi/2, -1.29490770291191 + pi/2, -1.25657773803303 + pi/2, -1.22353943055503 + pi/2, -1.19013328365251 + pi/2, -1.15935506991196 + pi/2, -1.1283015937438 + pi/2, -1.10302655579753 + pi/2, -1.07758232494542 + pi/2, -1.05502935975909 + pi/2, -1.03234779950079 + pi/2, -1.01571610929412 + pi/2, -0.999015604537713 + pi/2, -0.982245710839048 + pi/2, -0.965405584655155 + pi/2]
        self.theta3DH = [-1.49997139987598 + pi/2, -1.54390886030387 + pi/2, -1.58970272168809 + pi/2, -pi/2 + 1.50423950933982, -pi/2 + 1.45475461364956, -pi/2 + 1.41123965145139, -pi/2 + 1.36646123918297, -pi/2 + 1.32454245104741, -pi/2 + 1.28164197484252, -pi/2 + 1.24629776977495, -pi/2 + 1.21035842853485, -pi/2 + 1.17821924470232, -pi/2 + 1.14564851277058, -pi/2 + 1.12161199903286, -pi/2 + 1.09735817568203, -pi/2 + 1.07289197230526, -pi/2 + 1.04821802414523]
        self.theta4 = [-pi + 1.13597996001649, -pi + 1.17340525506298, -pi + 1.21267474179985, -pi + 1.25309988655013, -pi + 1.29424525468021, -pi + 1.32959273297925, -pi + 1.36505187498242, -pi + 1.39737534559634, -pi + 1.4295404508695, -pi + 1.45535255489327, -pi + 1.48092134447811, -pi + 1.50321055928462, -pi + 1.52521393527441, -pi + 1.54110206988095, -pi + 1.55679173905612, -1.56933105393382, -1.55410380859390] 
        self.theta5 = [0.519826496716735, 0.635522051921340, 0.750206056956972, 0.863884463612950, 0.976599864194937, 1.07178573180764, 1.16638777023008, 1.25224042268849, 1.33772567598098, 1.40654326457792, 1.47519711770368, 1.53556812065108, -1.54572961194842 + pi, -1.50176915337836 + pi, -1.45782751742871 + pi, -1.41389593296425 + pi, -1.36996587387532 + pi] 
        self.theta6 = [-1.08077225461884 + pi, -1.11131553769467 + pi, -1.14368086590173 + pi, -1.17731754865641 + pi, -1.21195394287464 + pi, -1.24205039365485 + pi, -1.27272714348319 + pi, -1.30117435186137 + pi, -1.33009222609625 + pi, -1.35373521147736 + pi, -1.37770710320355 + pi, -1.39907125460583 + pi, -1.42072171974764 + pi, -1.43662014193163 + pi, -1.45269210797159 + pi, -1.46894921440161 + pi, -1.48540501435286 + pi]
    
    def FK(self):
        if self.poses == 0:
            print("No poses in object")
            return
        
        s = { alpha0:     0, a0:     0, d1:  .75, q1: q1,
              alpha1: -pi/2, a1:   .35, d2:    0, q2: q2-pi/2,
              alpha2:     0, a2:  1.25, d3:    0, q3: q3,
              alpha3: -pi/2, a3: -.054, d4:  1.5, q4: q4,
              alpha4:  pi/2, a4:     0, d1:    0, q5: q5,
              alpha5: -pi/2, a5:     0, d1:    0, q6: q6,
              alpha6:     0, a6:  1.25, d7: .303, q7: 0,}

        t0_1 = Matrix([[0], [0], [d1 - 0.42]])
        t0_2 = Matrix([[a1*cos(q1)], [a1*sin(q1)], [d1]])
        t0_3 = Matrix([[(a1 + a2*sin(q2))*cos(q1)], [(a1 + a2*sin(q2))*sin(q1)], [a2*cos(q2) + d1]])
        t0_4 = Matrix([[(a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1) - 0.54*cos(q1)*cos(q2 + q3)], [(a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1) - 0.54*sin(q1)*cos(q2 + q3)], [a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) + 0.54*sin(q2 + q3)]])
        t0_5 = Matrix([[(a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)], [(a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)], [a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3)]])
        t0_6 = Matrix([[-0.193*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1) + 0.193*cos(q1)*cos(q5)*cos(q2 + q3)], [-0.193*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1) + 0.193*sin(q1)*cos(q5)*cos(q2 + q3)], [a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - 0.193*sin(q5)*cos(q4)*cos(q2 + q3) - 0.193*sin(q2 + q3)*cos(q5)]])
        t0_G = Matrix([[-d7*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)], [-d7*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)], [a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - d7*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5))]])
        T0_G = Matrix([
            [-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -d7*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*cos(q1)],
            [-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -d7*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2 + q3)) + (a1 + a2*sin(q2) + a3*sin(q2 + q3) + d4*cos(q2 + q3))*sin(q1)],
            [                                    -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3),                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                a2*cos(q2) + a3*cos(q2 + q3) + d1 - d4*sin(q2 + q3) - d7*(sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5))],
            [                                                                                       0,                                                                                                                                                            0,                                                                                                                                                            0,                                                                                                                                                             1]])

        transforms = [t0_1, t0_2, t0_3, t0_4, t0_5, t0_6, t0_G]

        sol_start = {q1: self.theta1[0], q2: self.theta2[0], q3: self.theta3DH[0], q4: self.theta4[0], q5: self.theta5[0], q6: self.theta6[0], 
                                     d1:  .75, a1:   .35, a2:  1.25, a3: -.054, d4:  1.5, d7: .303}

        length = self.poses-1

        sol_end = {q1: self.theta1[length], q2: self.theta2[length], q3: self.theta3DH[length], q4: self.theta4[length], q5: self.theta5[length], q6: self.theta6[length], 
                                     d1:  .75, a1:   .35, a2:  1.25, a3: -.054, d4:  1.5, d7: .303}

        self.kuka_start_x = [0]
        self.kuka_start_y = [0]
        self.kuka_start_z = [0]
        self.kuka_end_x = [0]
        self.kuka_end_y = [0]
        self.kuka_end_z = [0]

        for trans in transforms:
            self.kuka_start = trans.evalf(subs=sol_start)
            self.kuka_start_x.append(self.kuka_start[0])
            self.kuka_start_y.append(self.kuka_start[1])
            self.kuka_start_z.append(self.kuka_start[2])
            self.kuka_end = trans.evalf(subs=sol_end)
            self.kuka_end_x.append(self.kuka_end[0])
            self.kuka_end_y.append(self.kuka_end[1])
            self.kuka_end_z.append(self.kuka_end[2])
    
    def datadump(self):
        pickle.dump(self, open("datadump.dat", "ab"))
    
    def plot(self):
        plt.close()
        plt.style.use('fivethirtyeight')
        
        fig = plt.figure(figsize=(20, 14))
        mpl.rcParams['font.size'] = 16
        ax = fig.add_subplot(111, projection='3d', xlim=[-.5,2.5], ylim=[-.5,3.], zlim=[0,2.5]) 
        ax.view_init(elev=25, azim=210)
        
        marks = [1,2,3,4,5,6]
        
        ax.plot(self.kuka_start_x, self.kuka_start_y, self.kuka_start_z, c='grey', linestyle='--',
                marker='X', markersize = 12,  markevery=marks, label='start position')
        ax.plot(self.kuka_end_x, self.kuka_end_y, self.kuka_end_z, c='black', linestyle='--', 
                marker='X', markersize = 12, markevery=marks, label='end position')
        p = ax.scatter(self.Px, self.Py, self.Pz, c=self.error, cmap='Reds', marker='^',
                   depthshade=False, label='calculated gripper positions',
                       s = 100, vmin = 0, vmax = 10**-15)
        ax.legend()
        
        maxerr = max(self.error)
        maxpos = self.error.index(maxerr)
        xmax = self.Px[maxpos]
        ymax = self.Py[maxpos]
        zmax = self.Pz[maxpos]
    
    
        fig.colorbar(p, label='FK vs IK position error')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
    

        plt.savefig('test.png', bbox_inches='tight')
        plt.show()
        plt.close()
        
        return