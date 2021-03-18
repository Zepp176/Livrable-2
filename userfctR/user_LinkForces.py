# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020


def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    Flink = 0.0

    # Example: linear spring
    # k = 1000 #N/m
    # Z0= 0.1  #m
    # Flink = k*(Z-Z0)
    
    L1 = mbs_data.link_id['KXL1']
    L2 = mbs_data.link_id['KXL2']
    L3 = mbs_data.link_id['KXL3']
    L4 = mbs_data.link_id['KXL4']
    L5 = mbs_data.link_id['KXR1']
    L6 = mbs_data.link_id['KXR2']
    L7 = mbs_data.link_id['KXR3']
    L8 = mbs_data.link_id['KXR4']
    L9 = mbs_data.link_id['KYL1']
    L10 = mbs_data.link_id['KYL2']
    L11 = mbs_data.link_id['KYL3']
    L12 = mbs_data.link_id['KYL4']
    L13 = mbs_data.link_id['KYR1']
    L14 = mbs_data.link_id['KYR2']
    L15 = mbs_data.link_id['KYR3']
    L16 = mbs_data.link_id['KYR4']
    L17 = mbs_data.link_id['KDZL1']
    L18 = mbs_data.link_id['KDZL2']
    L19 = mbs_data.link_id['KDZL3']
    L20 = mbs_data.link_id['KDZL4']
    L21 = mbs_data.link_id['KDZR1']
    L22 = mbs_data.link_id['KDZR2']
    L23 = mbs_data.link_id['KDZR3']
    L24 = mbs_data.link_id['KDZR4']
    L25 = mbs_data.link_id['CKZL1']
    L26 = mbs_data.link_id['CKZL2']
    L27 = mbs_data.link_id['CKZR1']
    L28 = mbs_data.link_id['CKZR2']
    L29 = mbs_data.link_id['DZL1']
    L30 = mbs_data.link_id['DZL2']
    L31 = mbs_data.link_id['DZR1']
    L32 = mbs_data.link_id['DZR2']
    L33 = mbs_data.link_id['DYL1']
    L34 = mbs_data.link_id['DYL2']
    L35 = mbs_data.link_id['DYR1']
    L36 = mbs_data.link_id['DYR2']
    if identity in [L1, L2, L3, L4, L5, L6, L7, L8]:
       K  = mbs_data.user_model['kx1']['k'] *1000
       C  = mbs_data.user_model['kx1']['c'] *1000
       Z0 = mbs_data.user_model['kx1']['l'] /1000
       Flink = K*(Z-Z0)+C*Zd
    
    if identity in [L9, L10, L11, L12, L13, L14, L15, L16]:
       K  = mbs_data.user_model['ky1']['k'] *1000
       C  = mbs_data.user_model['ky1']['c'] *1000
       Z0 = mbs_data.user_model['ky1']['l'] /1000
       Flink = K*(Z-Z0)+C*Zd
       
    if identity in [L17, L18, L19, L20, L21, L22, L23, L24]:
       K  = (mbs_data.user_model['kz1']['k'] + mbs_data.user_model['dz1']['k']) *1000
       C  = (mbs_data.user_model['kz1']['c'] + mbs_data.user_model['dz1']['c']) *1000
       Z0 = mbs_data.user_model['kz1']['l'] /1000
       Flink = K*(Z-Z0)+C*Zd-4653.75*9.81 
    
    if identity in [L25, L26, L27, L28]:
       K  = mbs_data.user_model['kz2']['k'] *1000
       C  = mbs_data.user_model['kz2']['c'] *1000
       Z0 = mbs_data.user_model['kz2']['l'] /1000
       Flink = K*(Z-Z0)+C*Zd-8000*9.81
       
    if identity in [L29, L30, L31, L32]:
       K  = mbs_data.user_model['dz2']['k'] *1000
       C  = mbs_data.user_model['dz2']['c'] *1000
       Z0 = mbs_data.user_model['dz2']['l'] /1000
       Flink = K*(Z-Z0)+C*Zd-8000*9.81
    
    if identity in [L33, L34, L35, L36]:
       K  = mbs_data.user_model['dy2']['k'] *1000
       C  = mbs_data.user_model['dy2']['c'] *1000
       Z0 = mbs_data.user_model['dy2']['l'] /1000
       Flink = K*(Z-Z0)+C*Zd

    return Flink
