from math import asin, cos, sin, pi, sqrt

def defect_position(traj_coord, s):
    """Add the perturbation of the defect to the trajectory.
    Here an example of cosine longitudinal level.
    Parameters
    ----------
    traj_coord : numpy.ndarray
    Coordinates of the track at the curviline value 's'.
    It contains : [6, x, y, z, yaw, pitch, roll]
    Each value impacted by the defect must be incremented.
    s : float
    Current curviline value.
    Returns
    -------
    None.
    Notes
    -----
    The coordinates in traj_coord are:
    - x: value of the T1 joint of the track;
    - y: value of the T2 joint of the track;
    - z: value of the T3 joint of the track;
    - yaw: value of the R3 joint of the track;
    - pitch: value of the R2 joint of the track;
    - roll: value of the R1 joint of the track;
    for the given curviline value.
    """
    
    if s < 30.:
        return
    
    traj_coord[2] += 0.01 * cos(pi * (s - 30.) / 5.) - 0.01
    dt2_ds = -2e-3 * pi * sin(pi * (s - 30.) / 5.)
    traj_coord[4] += asin(dt2_ds)



def defect_jac(mbs_data, s, jac, id_h, id_q_s):
    """Add the perturbation of the defect to the constraint Jacobian.
    Here an example of cosine longitudinal level.
    Parameters
    ----------
    mbs_data: MBsysPy.MbsData
    The multibody main class instance.
    s : float
    Current curviline valuebe in jac[id_h:id_h+6, id_q_s]
    id_h : int.
    jac : numpy.ndarray
    The Jacobian of all user constraints.
    The rows relative to this track constraints are: [id_h:id_h+6].
    The column relative to this track curviline coordinate is [id_q_s].
    The elements that can be modified should
    The index of the user constraint relative to the track x position.
    id_q_s : int
    The index of this track curviline value in mbs_data.q.
    Returns
    -------
    None.
    """
    
    if s < 30.:
        return
    
    f_sin = sin(pi * (s - 30.) / 5.)
    f_cos = cos(pi * (s - 30.) / 5.)
    
    dt2_ds = -2e-3 * pi * f_sin
    jac[id_h + 1, id_q_s] -= dt2_ds
    
    ddt2_dsds = -0.01 * pi * pi * f_cos / 25.
    dr3_ds_den = sqrt(1. - dt2_ds * dt2_ds)
    
    dr3_ds = ddt2_dsds / dr3_ds_den
    jac[id_h + 3, id_q_s] += -dr3_ds



def defect_jdqd(mbs_data, s, jdqd, id_h, id_q_s):
    """Add the perturbation of the defect to the constraint jdqd term.
    Here an example of cosine longitudinal level.
    Parameters
    ----------
    mbs_data: MBsysPy.MbsData
    The multibody main class instance.
    s : float
    Current curviline value.
    jdqd : numpy.ndarray
    The jdqd term of all user constraints.
    The element relative to this track constraints are: [id_h:id_h+6].
    id_h : int
    The index of the user constraint relative to the track x position.
    id_q_s : int
    The index of this track curviline value in mbs_data.q.
    Returns
    -------
    None.
    """
    
    if s < 30.:
        return
    
    sd = mbs_data.qd[id_q_s]
    f_sin = sin(pi * (s - 30.) / 5.)
    f_cos = cos(pi * (s - 30.) / 5.)
    
    ddt2_dsds = -4e-4 * pi * pi * f_cos
    
    ddt2_dsdt = ddt2_dsds * sd
    jdqd[id_h + 2] += -ddt2_dsdt * sd
    
    dt2_ds = -2e-3 * pi * f_sin
    dddt2_dsdsdt = 8e-5 * pi * pi * pi * f_sin * sd
    
    ddr3_dsdt_num1 = ddt2_dsds * dt2_ds * ddt2_dsdt
    ddr3_dsdt_num2 = -(1. - dt2_ds * dt2_ds) * dddt2_dsdsdt
    ddr3_dsdt_den = (1. - dt2_ds * dt2_ds) * sqrt(1. - dt2_ds * dt2_ds)
    
    ddr3_dsdt = -(ddr3_dsdt_num1 + ddr3_dsdt_num2) / ddr3_dsdt_den
    jdqd[id_h + 4] += -ddr3_dsdt * sd