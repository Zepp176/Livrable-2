#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the direct dynamic module (time integration of equations of motion).
 - if available, plot the time evolution of the first generalized coordinate.

It may have to be adapted and completed by the user.


Universite catholique de Louvain
CEREM : Centre for research in mechatronics

http://www.robotran.eu
Contact : info@robotran.be

(c) Universite catholique de Louvain
"""

# %%============================================================================
# Packages loading
# =============================================================================

import MBsysPy as Robotran

import os   
import mbs_rwt as rwt
import mbs_rwc as rwc
from defauts_voie import defect_position, defect_jac, defect_jdqd

voie = 2


# creating trajectory
if voie == 1: # ligne droite avec défauts
    print("ligne droite avec défauts")
    horiz_def = [{'geom': 'li', 'L': 200.}]
    cant_def_ideal = [{'L': 200., 'cant_0':0.0, 'cant_f': 0.0}]
    
elif voie == 2: # courbe parfaite
    print("courbe parfaite")
    horiz_def = [{'geom': 'li', 'L': 40.},
                 {'geom': 'cl', 'L': 20.},
                 {'geom': 'cu', 'L': 120., 'R': 5000.},
                 {'geom': 'cl', 'L':20.}]
    cant_def_ideal = [{'L': 40., 'cant_0':0.0, 'cant_f': 0.0},
                      {'L': 20., 'cant_f': 0.0015},
                      {'L': 120., 'cant_f': 0.0015},
                      {'L':20., 'cant_f': 0.}]
    
elif voie == 3: # ligne droite parfaite
    print("ligne droite parfaite")
    horiz_def = [{'geom': 'li', 'L': 200.}]
    cant_def_ideal = [{'L': 200., 'cant_0':0.0, 'cant_f': 0.0}]

traj_folder = "."
horiz_file = "Manchester_horizontal"
cant_file = "Manchester_cant_ideal"
elev_file = None

horiz_traj = rwt.PlaneTrajectory(horiz_def, 5e-3)
horiz_traj.to_file(path = traj_folder, plane_name = horiz_file)

cant_traj = rwt.CantTrajectory(cant_def_ideal)
cant_traj.to_file(path = traj_folder, cant_name = cant_file)

rwt.generate_vrml('../animationR/vrml/trajectoire', horiz_traj, traj_cant = cant_traj,
                  mbs_filename = '../dataR/one.mbs', replace = True, backup = False)

# %%===========================================================================
# Project loading
# =============================================================================
mbs_data = Robotran.MbsData('../dataR/one.mbs')


#configuration
hori_path = None
cant_path = None
elev_path = None

if horiz_file:
    hori_path = os.path.abspath(os.path.join(traj_folder, horiz_file))
if cant_file:
    cant_path = os.path.abspath(os.path.join(traj_folder, cant_file))
if elev_file:
    elev_path = os.path.abspath(os.path.join(traj_folder, elev_file))
    
track = rwt.RwtTrackGeometry(mbs_data, plane_filename = hori_path,
                             elevation_filename = elev_path,
                             cant_filename = cant_path,
                             gauge = 1.435)

track.set_followers([mbs_data.joint_id['R1_Track1'],
                     mbs_data.joint_id['R1_Track2'],
                     mbs_data.joint_id['R1_Track3'],
                     mbs_data.joint_id['R1_Track4']],
                    [mbs_data.joint_id['T1_Track_curv1'],
                     mbs_data.joint_id['T1_Track_curv2'],
                     mbs_data.joint_id['T1_Track_curv3'],
                     mbs_data.joint_id['T1_Track_curv4']],1)

track.set_user_model(mbs_data, 'addons', 'rwt')

if voie == 1:
    track.set_defect('vertical', defect_position, defect_jac, defect_jdqd)

last_used_constraint = 6 * track.nb_followers

#configuration contact
config = {'nb_wheelsets': 4,
          'gauge': 1.435 + 0.072,
          'rail_radius': 0.3,
          'rail_poisson': 0.28,
          'rail_young': 2.02752e+11,
          'wheel_radius': 0.46,
          'wheel_poisson': 0.28,
          'wheel_young': 2.02752e+11,
          'conicity_angle': 0.15,
          'friction': 0.4,
          'saturation': 2,
          'with_railway_track': 1,
          'verbose': 2}

indices = {'first_lambda_id': last_used_constraint +1,
           'wheelsets_ref_ids': [mbs_data.joint_id['R1_Track1'],
                                 mbs_data.joint_id['R1_Track2'],
                                 mbs_data.joint_id['R1_Track3'],
                                 mbs_data.joint_id['R1_Track4']],
           'wheels_sensor_ids': [mbs_data.extforce_id['EFL1'], mbs_data.extforce_id['EFR1'],
                                 mbs_data.extforce_id['EFL2'], mbs_data.extforce_id['EFR2'],
                                 mbs_data.extforce_id['EFL3'], mbs_data.extforce_id['EFR3'],
                                 mbs_data.extforce_id['EFL4'], mbs_data.extforce_id['EFR4']]}

rwc_main = rwc.RwcMain(configuration = config, indices = indices, set_computation = True)

rwc_main.set_user_model(mbs_data, 'addons', 'rwc')

last_used_constraint += 2 * rwc_main.nb_wheelsets

mbs_data.set_nb_userc(last_used_constraint+6)
# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# %%===========================================================================
# Direct Dynamics
# =============================================================================
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=20.0, save2file=1)
results = mbs_dirdyn.run()

# %%===========================================================================
# Plotting results
# =============================================================================
try:
    import matplotlib.pyplot as plt
except Exception:
    raise RuntimeError('Unable to load matplotlib, plotting results unavailable.')

if voie == 1:
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_caisse1']]+results.q[:,mbs_data.joint_id['T2_chassis1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('mouvement latéral de la caisse (T2) [m]')
    axis.set_title('Voie rectiligne avec défaut de dressage')
    plt.savefig("T2_caisse_rect_def.svg", format='svg')
    
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_Essieu1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('mouvement latéral d\'un essieu (T2) [m]')
    axis.set_title('Voie rectiligne avec défaut de dressage')
    plt.savefig("T2_essieu_rect_def.svg", format='svg')
    
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['R1_caisse1']]+results.q[:,mbs_data.joint_id['R1_chassis1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('angle de roulis (R1) [rad]')
    axis.set_title('Voie rectiligne avec défaut de dressage')
    plt.savefig("R1_caisse_rect_def.svg", format='svg')
    
if voie == 2:
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_caisse1']]+results.q[:,mbs_data.joint_id['T2_chassis1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('mouvement latéral de la caisse (T2) [m]')
    axis.set_title('Voie courbe parfaite')
    plt.savefig("T2_caisse_curv_parf.svg", format='svg')
    
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['R1_caisse1']]+results.q[:,mbs_data.joint_id['R1_chassis1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('angle de roulis (R1) [rad]')
    axis.set_title('Voie courbe parfaite')
    plt.savefig("R1_caisse_curv_parf.svg", format='svg')
    
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_Track1']], label='q[1]')
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_Track3']], label='1qdsq')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('angle de roulis (R1) [rad]')
    axis.set_title('tracks')
    plt.savefig("tracks.svg", format='svg')

if voie == 3:
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_caisse1']]+results.q[:,mbs_data.joint_id['T2_chassis1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('mouvement latéral de la caisse (T2) [m]')
    axis.set_title('Voie rectiligne parfaite')
    plt.savefig("T2_caisse_rect_parf.svg", format='svg')
    
    fig = plt.figure()
    axis = fig.gca()
    axis.plot(results.q[:, 0], results.q[:, mbs_data.joint_id['T2_caisse1']]+results.q[:,mbs_data.joint_id['T2_chassis1']], label='q[1]')
    axis.grid(True)
    axis.set_xlim(left=mbs_dirdyn.get_options('t0'), right=mbs_dirdyn.get_options('tf'))
    axis.set_xlabel('temps [s]')
    axis.set_ylabel('angle de roulis (R1) [rad]')
    axis.set_title('Voie rectiligne parfaite')
    plt.savefig("R1_caisse_rect_parf.svg", format='svg')

plt.show()