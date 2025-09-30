import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.spatial.transform import Rotation as R
from earth import *

def altitude_update(x_prior, wib, dt):
    '''
    Updates the attitude using the feedback architecture.
    Inputs: 
        x_prior (np.ndarray): A 15x1 numpy array representing the prior state.
        wib (np.ndarray): A 3x1 numpy array representing gyro measurement.
        dt (float): Time step.
    Outputs: 
        Prior Rotation Matrix and Updated Rotation Matrix.
    '''

    # Subtract the bias from the gyro measurement
    wib = wib - x_prior[12:15,:]

    # Earths rate of rotation (rad/s)
    we = 7.2921157E-5 

    # wie is the skew symmetric matrix of we
    omegaie = np.array([[0, we, 0], [-we, 0, 0], [0, 0, 0]])

    Rn, Re, Rp = principal_radii(x_prior[0,0], x_prior[2,0])

    wen = np.array([x_prior[7,0] / (Re + x_prior[2,0]), -x_prior[6,0] / (Rn + x_prior[2,0]), -x_prior[7,0] * np.tan(x_prior[0,0]) / (Re + x_prior[2,0])]).reshape(-1,1)
    
    omegaen = np.array([[0, -wen[2,0], wen[1,0]], [wen[2,0], 0, -wen[0,0]], [-wen[1,0], wen[0,0], 0]])

    omegaib = np.array([[0, -wib[2,0], wib[1,0]], [wib[2,0], 0, -wib[0,0]], [-wib[1,0], wib[0,0], 0]])

    # Compute the prior rotation matrix using the angles from the prior state
    Rbtminusonen = R.from_euler('xyz', x_prior[3:6,0].T).as_matrix()

    # Compute the updated rotation matrix
    Rbtn = Rbtminusonen @ (np.eye(3) + omegaib * dt) - (omegaie + omegaen) @ Rbtminusonen * dt

    return Rbtminusonen, Rbtn

def velocity_update(x_prior, R_updated, R_prior, fb, dt):
    '''
    Updates the velocity using the feedback architecture.
    Inputs: 
        x_prior (np.ndarray): A 15x1 numpy array representing the prior state.
        R_updated (np.ndarray): A 3x3 numpy array representing the updated rotation matrix.
        R_prior (np.ndarray): A 3x3 numpy array representing the prior rotation matrix.
        fb (np.ndarray): A 3x1 numpy array representing the accelerometer measurement.
        dt (float): Time step.
    Outputs: 
        Updated Velocity (np.ndarray): A 3x1 numpy array representing the updated velocity.
    '''

    # Subtract the bias from the accelerometer measurement
    fb = fb - x_prior[9:12,:]

    fnt = 0.5 * (R_updated + R_prior) @ fb

    # Repeat some steps from altitude update
    # Earths rate of rotation (rad/s)
    we = 7.2921157E-5 

    # wie is the skew symmetric matrix of we
    omegaie = np.array([[0, we, 0], [-we, 0, 0], [0, 0, 0]])

    Rn, Re, Rp = principal_radii(x_prior[0,0], x_prior[2,0])

    wen = np.array([x_prior[7,0] / (Re + x_prior[2,0]), -x_prior[6,0] / (Rn + x_prior[2,0]), -x_prior[7,0] * np.tan(x_prior[0,0]) / (Re + x_prior[2,0])]).reshape(-1,1)
    
    omegaen = np.array([[0, -wen[2,0], wen[1,0]], [wen[2,0], 0, -wen[0,0]], [-wen[1,0], wen[0,0], 0]])

    g_LH = gravity_n(x_prior[0,0], x_prior[2,0]).reshape(-1,1)

    vnt = x_prior[6:9,0].reshape(-1,1) + dt*(fnt + g_LH - (omegaen + 2*omegaie)@x_prior[6:9,0].reshape(-1,1))

    return vnt

def position_update(x_prior, vnt, dt):
    '''
    Updates the position using the feedback architecture.
    Inputs: 
        x_prior (np.ndarray): A 15x1 numpy array representing the prior state.
        vnt (np.ndarray): A 3x1 numpy array representing the updated velocity.
    Outputs: 
        (np.ndarray): A 3x1 numpy array representing the updated latitude, longitude, and altitude.
    '''

    # Compute the updated altitude
    ht = x_prior[2,0] - 0.5 * dt * (x_prior[8,0] + vnt[2,0])

    Rn, Re, Rp = principal_radii(x_prior[0,0], x_prior[2,0])

    Lat_updated = x_prior[0,0] + 0.5 * dt * (x_prior[6,0] / (Rn + x_prior[2,0]) + vnt[0,0] / (Rn + ht))

    Rn_updated, Re_updated, Rp_post = principal_radii(Lat_updated, ht)

    Lon_updated = x_prior[1,0] + 0.5 * dt * (x_prior[7,0] / ((Re + x_prior[2,0])*np.cos(np.deg2rad(x_prior[0,0]))) + vnt[1,0]/(Re_updated + ht)*np.cos(np.deg2rad(Lat_updated)))
    
    return Lat_updated, Lon_updated, ht


def feedback_propagation_model(x_prior, gyro, acc, dt):
    '''
    Propagates the state using the feedback architecture.
    Inputs: 
        x_prior (np.ndarray): A 15x1 numpy array representing the prior state.
        gyro (np.ndarray): A 3x1 numpy array representing gyro measurement.
        acc (np.ndarray): A 3x1 numpy array representing accelerometer measurement.
        dt (float): Time step.
    Outputs: 
        (np.ndarray): A 15x1 numpy array representing the updated state.
    '''

    # Update the attitude
    Rbtminusonen, Rbtn = altitude_update(x_prior, gyro, dt)

    # Update the velocity
    vnt = velocity_update(x_prior, Rbtn, Rbtminusonen, acc, dt)

    # Update the position
    lat, lon, alt = position_update(x_prior, vnt, dt)

    # Update the bias
    acc_bias = x_prior[9:12,:]
    gyro_bias = x_prior[12:15,:]

    #Get the roll, pitch and yaw angles
    qt = R.from_matrix(Rbtn).as_euler('xyz', degrees=True).reshape(-1,1)

    x_updated = np.vstack([lat, lon, alt, qt, vnt, acc_bias, gyro_bias])

    return x_updated





     