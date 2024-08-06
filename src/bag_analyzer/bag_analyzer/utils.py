import numpy as np
import quaternion



def interp(x, xp, fp):
    if len(fp.shape) == 1:
        return np.interp(x, xp, fp)
    
    if len(fp.shape) == 2:
        return np.array([
            np.interp(x, xp, fp[:, i]) for i in range(fp.shape[1])
        ]).transpose()

def quat2rpy(q):
    """
    Convert a numpy.quaternion or an array of numpy.quaternion into roll-pitch-yaw angles.
    """
    
    if q.size == 1:
        w = q.w
        x = q.x
        y = q.y
        z = q.z
        
        r = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        p = np.arcsin(2*(w*y - z*x))
        y = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        
        return np.array([r, p, y])

    if q.size > 1:
        rpy = np.zeros((0, 3))
        
        for i in range(q.size):
            w = q[i].w
            x = q[i].x
            y = q[i].y
            z = q[i].z

            r = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
            p = np.arcsin(2*(w*y - z*x))
            y = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

            rpy = np.vstack((rpy, np.array([r, p, y])))
            
        return rpy
    
def quat_rot(q, v):
    """
    Rotate a three dimensional vector v with quaternion q.
    
    params
    ----
    q: quaternion
    v: vector to be rotated
    
    return
    ----
    q * [0, v] * q.conj
    """
    
    return quaternion.as_vector_part(
        q * quaternion.from_vector_part(v) * q.conj()
    )