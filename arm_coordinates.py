import numpy as np

def end_point(lcg, h_platform, side, length, alpha, beta):
    if side == "positive":
        end_point = np.array([length*np.cos(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                                length*np.sin(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                                length*np.sin(np.deg2rad(beta))+ lcg])
    elif side == "negative":
        end_point = np.array([length*np.cos(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                                length*np.sin(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                                -length*np.sin(np.deg2rad(beta)) - (h_platform - lcg)])
    return end_point[0], end_point[1], end_point[2]

def length_and_angle(end_point, attachment_point):
    length = np.linalg.norm(end_point-attachment_point)
    alpha = np.degrees(np.arctan2(end_point[1]-attachment_point[1], end_point[0]-attachment_point[0]))
    beta = np.degrees(np.arcsin((end_point[2]-attachment_point[2])/length))
    return length, alpha, beta