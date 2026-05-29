import numpy as np


class State6DOF:
    '''Immutable snapshot of state for the 6DOF model.'''

    def __init__(self, 
                 pos_e, 
                 vel_e,
                 vel_b,
                 euler, 
                 euler_dot,
                 DCM_to_e, 
                 DCM_to_b
                 ):
        self.pos_e = pos_e
        self.vel_e = vel_e
        self.vel_b = vel_b
        self.euler = euler
        self.euler_dot = euler_dot
        self.DCM_to_e = DCM_to_e
        self.DCM_to_b = DCM_to_b


class Euler6DOFModel:
    '''
    Model linear and rotational kinematics of a 3D rigidbody residing 
    in a flat, inertial reference frame (i.e "earth"). Integrates rotational kinematics
    in Euler angles, so beware of gimble lock. Note: the "earth" and body coordinates
    will both have the same origin.
    '''

    def __init__(self, pos_e0, vel_b0, euler0, euler_dot0):
        ''' IMPORTANT: All vectors must be given as row vectors.
        :param numpy.array pos_e0: Initial position of body in "earth" frame.
        :param numpy.array vel_b0: Initial velocity of body in "earth" frame.
        :param numpy.array euler0: Initial roll, pitch, raw of body in euler angles (rad).
        :param numpy.array euler_dot0: Initial rate of roll, pitch, yaw of body in euler angles (rad).
        '''
        self.pos_e = pos_e0
        self.vel_b = vel_b0
        self.euler = euler0
        self.euler_dot = euler_dot0
        
        self.DCM_to_e = self._getDCMToEarth()
        self.DCM_to_b = self.DCM_to_e.T

    def _getDCMToEarth(self):
        '''Get body frame -> earth frame transformation matrix (direction cosine matrix).'''
        xrot, yrot, zrot = self.euler
        return np.array([
            [np.cos(yrot)*np.cos(zrot), np.sin(xrot)*np.sin(yrot)*np.cos(zrot) - np.cos(xrot)*np.sin(zrot), np.cos(xrot)*np.sin(yrot)*np.cos(zrot) + np.sin(xrot)*np.sin(zrot)],
            [np.cos(yrot)*np.sin(zrot), np.sin(xrot)*np.sin(yrot)*np.sin(zrot) + np.cos(xrot)*np.cos(zrot), np.cos(xrot)*np.sin(yrot)*np.sin(zrot) - np.sin(xrot)*np.cos(zrot)],
            [-np.sin(yrot), np.sin(xrot)*np.cos(yrot), np.cos(xrot)*np.cos(yrot)]
        ])
        
    def update(self, F_b, T_b, m, dmdt, I_b, dIdt_b, dt):
        ''' Progress 6DOF model by dt time.
        :param numpy.array F_b: Net force (in body frame) being applied to body.
        :param numpy.array T_b: Net torque (in body frame) being applied to body.
        :param float m: Body mass at this instant.
        :param float dmdt: Body mass rate of change.
        :param numpy.array I_b: 3x3 moment of inertia matrix in body frame.
        :param numpy.array dIdt_b: Moment of inertia rate of change (body frame).
        :param float dt: Time step
        '''
        # Get linear and angular acceleration
        acc_b = F_b / m - np.cross(self.euler_dot, self.vel_b)
        euler_dotdot = np.linalg.inv(I_b) @ (T_b - np.cross(self.euler_dot, I_b @ self.euler_dot) - dIdt_b @ self.euler_dot)

        # Integrate angles
        self.euler_dot += euler_dotdot*dt
        self.euler += self.euler_dot*dt

        # Get transformations
        self.DCM_to_e = self._getDCMToEarth()
        self.DCM_to_b = self.DCM_to_e.T

        # Integrate velocities from body frame
        self.vel_b += acc_b*dt
        vel_e = self.DCM_to_e @ self.vel_b

        # Integrate "earth" position and mass 
        self.pos_e += vel_e*dt
        m += dmdt*dt
        
        return State6DOF(self.pos_e, vel_e, self.vel_b, self.euler, self.euler_dot, self.DCM_to_e, self.DCM_to_b)