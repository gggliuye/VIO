import glob
import os
import numpy as np
import random
import math

import open3d as o3d
from pyntcloud import PyntCloud
import matplotlib.pyplot as plt

def read_modelnet40_data(file_name):
    points_list = []
    with open(file_name, 'r') as file_to_read:
        while True:
            lines = file_to_read.readline()
            if not lines:
                break
                pass
            x = lines.split(',')[0]
            y = lines.split(',')[1]
            z = lines.split(',')[2]
            points_list.append(np.array([x,y,z]))
    points_array = np.asarray(points_list)
    #print("points array shape : ", points_array.shape)
    pts = points_array.astype(np.float32)
    return pts


def get_random_cloud():
    rand_idx = random.randint(0,len(files_cloud)-1)
    file_name = path_modelnet40 + files_cloud[rand_idx]
    pts = read_modelnet40_data(file_name)
    print('load points from ', file_name)
    return pts

def PCA_threshold(covariance, lambda_12, lambda_32):
    # SVD
    U,sigma,VT = np.linalg.svd(covariance)

    #lambda_12 = 0.4
    #lambda_32 = 0.4
    thre_12 = sigma[1]/sigma[0]
    thre_32 = sigma[2]/sigma[1]
    ret = True
    if (sigma[2] < 0.0001):
        ret = False

    if(thre_12 > lambda_12):
        ret = False
    if (thre_32 > lambda_32):
        ret = False
    #print(thre_12, thre_32)
    return ret


def covariance_of_cloud(center_pt, cloud, weights):
    delta = cloud - center_pt
    covariance = np.dot(delta.transpose() , np.tile(weights, (3, 1)).transpose() * delta)
    covariance = covariance / np.sum(weights)
    return covariance

def weight_sparsity(cloud, radius):
    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud[:,0:3])
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    weights = np.ones(cloud.shape[0])
    for i in range(cloud.shape[0]):
        current_pt = cloud[i, :]
        [k, idx, _] = pcd_tree.search_radius_vector_3d(current_pt, radius)

        if(k < 1):
            continue
        weights[i] = 1/k
    return weights

def SSI_cloud(cloud, lambda_12=0.4, lambda_32=0.4, if_show=True):
    search_radius = 0.5

    #weights = weight_sparsity(cloud, search_radius)
    weights = np.ones(cloud.shape[0])

    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud[:,0:3])
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    features = []
    show_spheres = []
    for i in range(cloud.shape[0]):
        current_pt = cloud[i, :]
        [k, idx, _] = pcd_tree.search_radius_vector_3d(current_pt, search_radius)
        #[k, idx, _] = pcd_tree.search_knn_vector_3d(current_pt, 60)

        if(k < 5):
            continue

        neighbor_points = []
        weights_neighbor = np.ones(k)
        for j in range(k):
            neighbor_points.append(cloud[idx[j],:])
            weights_neighbor[j] = weights[idx[j]]
        #print(k)
        covariance = covariance_of_cloud(current_pt, np.array(neighbor_points), weights_neighbor)

        if(PCA_threshold(covariance, lambda_12, lambda_32)):
            features.append(current_pt)
            if(if_show):
                mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.2)
                mesh_sphere.paint_uniform_color([1, 0, 0])
                mesh_sphere.translate(current_pt.tolist())
                show_spheres.append(mesh_sphere)
    return np.array(features), show_spheres


def estimate_normal(cloud, neighbor_radius = 0.05):

    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud[:,0:3])
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    normals = []
    for i in range(cloud.shape[0]):
        current_pt = cloud[i, :]
        [k, idx, _] = pcd_tree.search_radius_vector_3d(current_pt, neighbor_radius)
        #[k, idx, _] = pcd_tree.search_knn_vector_3d(current_pt, 60)

        # calculate the normals
        neighbor_points = []
        for j in range(k):
            neighbor_points.append(cloud[idx[j],:])

        if k < 4:
            point_cloud_normal = np.array([0,0,0])
        else :
            U,eigenvalues,VT = np.linalg.svd(np.asarray(neighbor_points))
            eigenvectors = np.transpose(VT)
            point_cloud_normal = eigenvectors[:, 2]

        normals.append(point_cloud_normal)
    return np.asarray(normals)

def calculate_and_flat_histograms(data_features, histogram_size = 10):
    # alpha in range(-1,1)
    # phi in range(-1,1)
    # theta in range(-pi/2, pi/2) - > normalize to -1, 1

    # step 1. normalize all of them to range(0,1)
    data_tmp = (data_features + 1)/2
    #histogram_size = 10
    interval = 1 / (histogram_size)

    result = np.zeros(histogram_size*histogram_size*histogram_size)

    for i in range(data_tmp.shape[0]):
        current_param = data_tmp[i,:]
        id_param = np.zeros(3)
        for n in range(3):
            id_param[n] = int(current_param[n] / interval)
            if(id_param[n] >= histogram_size):
                id_param[n] = histogram_size-1
        id_result = int(id_param[0] * histogram_size * histogram_size + id_param[1] * histogram_size + id_param[2])
        result[id_result] += 1

    return result

def calculate_three_histograms(data_features, histogram_size = 10):
    # alpha in range(-1,1)
    # phi in range(-1,1)
    # theta in range(-pi/2, pi/2) - > normalize to -1, 1

    # step 1. normalize all of them to range(0,1)
    data_tmp = (data_features + 1)/2
    #histogram_size = 10
    interval = 1 / (histogram_size)

    result = np.zeros([3, histogram_size])

    for i in range(data_tmp.shape[0]):
        current_param = data_tmp[i,:]
        id_param = np.zeros(3)
        for n in range(3):
            if(math.isnan(current_param[n])):
                print(i, n)
                continue
            id_param = int(current_param[n] / interval)
            if(id_param >= histogram_size):
                id_param = histogram_size-1
            result[n, id_param] += 1

    return result


def SPFH_descriptors(cloud, cloud_normal, features):
    #search_radius = 0.05
    histogram_size = 5
    search_k = 20

    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud[:,0:3])
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    descriptors = []
    for i in range(features.shape[0]):
        current_pt = features[i, :]
        #[k, idx, _] = pcd_tree.search_radius_vector_3d(current_pt, search_radius)
        [k, idx, _] = pcd_tree.search_knn_vector_3d(current_pt, search_k)

        # calculate the parameters
        dir_u = cloud_normal[i]
        parameters_feaure = np.zeros([k, 3])
        for j in range(k):
            neighbor_point = cloud[idx[j],:]
            neighbor_normal = cloud_normal[idx[j],:]

            param_d = np.linalg.norm(neighbor_point - current_pt)
            if(param_d < 10e-5):
                continue
            vec_d = (neighbor_point - current_pt) / param_d
            dir_v = np.cross(dir_u, vec_d)
            dir_w = np.cross(dir_u, dir_v)

            param_alpha = np.dot(dir_v, neighbor_normal)
            param_phi = np.dot(dir_u, vec_d)
            if(np.dot(dir_u, neighbor_normal) < 10e-4):
                param_theta = np.pi / 2
            else:
                tan_theta = np.dot(dir_w, neighbor_normal) / np.dot(dir_u, neighbor_normal)
                param_theta = np.arctan(tan_theta)

            # normalize theta to range(-1,1)
            param_theta = param_theta/(np.pi/2)
            parameters_feaure[j, :] = [param_alpha, param_phi, param_theta]

        # calculate the histograms
        hist = calculate_three_histograms(parameters_feaure, histogram_size)
        descriptors.append(hist)
    return descriptors

def FPFH_descriptors(cloud, cloud_normal, features):
    #search_radius = 0.05
    histogram_size = 5
    search_k = 20

    point_cloud_o3d = o3d.geometry.PointCloud()
    point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud[:,0:3])
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    descriptors = []
    for i in range(features.shape[0]):
        current_pt = features[i, :]
        #[k, idx, _] = pcd_tree.search_radius_vector_3d(current_pt, search_radius)
        [k, idx, _] = pcd_tree.search_knn_vector_3d(current_pt, search_k)

        # calculate the parameters
        dir_u = cloud_normal[i]
        parameters_feaure = np.zeros([k, 3])
        weighted_neighbor_hist = np.zeros([3, histogram_size])
        for j in range(k):
            neighbor_point = cloud[idx[j],:]
            neighbor_normal = cloud_normal[idx[j],:]

            param_d = np.linalg.norm(neighbor_point - current_pt)
            if(param_d < 10e-5):
                parameters_query = np.zeros(3)
                weight = 0
            else:
                weight = 1 / param_d
                vec_d = (neighbor_point - current_pt) / param_d
                dir_v = np.cross(dir_u, vec_d)
                dir_w = np.cross(dir_u, dir_v)

                param_alpha = np.dot(dir_v, neighbor_normal)
                param_phi = np.dot(dir_u, vec_d)
                if(np.dot(dir_u, neighbor_normal) < 10e-3):
                    param_theta = np.pi / 2
                else:
                    tan_theta = np.dot(dir_w, neighbor_normal) / np.dot(dir_u, neighbor_normal)
                    param_theta = np.arctan(tan_theta)

                # normalize theta to range(-1,1)
                param_theta = param_theta/(np.pi/2)
                parameters_query = np.array([param_alpha, param_phi, param_theta])

            #calculate neighbors' SPFH
            [k, idx_neighbor, _] = pcd_tree.search_knn_vector_3d(neighbor_point, search_k)
            parameters_neighbors = np.zeros([k, 3])
            for m in range(k):
                neighbor_neighbor_point = cloud[idx_neighbor[m],:]
                neighbor_neighbor_normal = cloud_normal[idx_neighbor[m],:]

                param_d = np.linalg.norm(neighbor_point - neighbor_neighbor_point)
                if(param_d < 10e-5):
                    continue
                vec_d = (neighbor_neighbor_point - neighbor_point) / param_d
                dir_v = np.cross(neighbor_normal, vec_d)
                dir_w = np.cross(neighbor_normal, dir_v)

                param_alpha = np.dot(dir_v, neighbor_normal)
                param_phi = np.dot(neighbor_normal, vec_d)
                if(np.dot(neighbor_normal, neighbor_neighbor_normal) < 10e-6):
                    param_theta = np.pi / 2
                else:
                    tan_theta = np.dot(dir_w, neighbor_neighbor_normal) / np.dot(neighbor_normal, neighbor_neighbor_normal)
                    param_theta = np.arctan(tan_theta)

                # normalize theta to range(-1,1)
                param_theta = param_theta/(np.pi/2)
                parameters_neighbors[m,:] = [param_alpha, param_phi, param_theta]

            hist = calculate_three_histograms(parameters_neighbors, histogram_size)
            weighted_neighbor_hist += weight * hist

        # calculate the histograms
        hist = calculate_three_histograms(parameters_feaure, histogram_size) + weighted_neighbor_hist / search_k
        descriptors.append(hist)
    return descriptors

def ReformDescriptors(descriptors):
    reformed = []
    for i in range(len(descriptors)):
        reformed.append(descriptors[i].reshape(descriptors[i].shape[1]*3))
    return np.asarray(reformed)

def MatchFPFH(descriptors_first, descriptors_second):
    descriptors_1 = ReformDescriptors(descriptors_first)
    descriptors_2 = ReformDescriptors(descriptors_second)
    matches = []
    distances = []
    distance_threshold = 0.5
    for i in range(descriptors_1.shape[0]):
        distance_12 =  np.linalg.norm(descriptors_2 - descriptors_1[i,:],axis = 1)
        match_id = np.argmin(distance_12)
        #print(distance_12[match_id])
        #if(distance_12[match_id] < distance_threshold):
        matches.append(np.array([i, match_id]))
        distances.append(distance_12[match_id])
    return np.asarray(matches), np.asarray(distances)

def MatchFPFH_twoway(descriptors_first, descriptors_second):
    matches_12, distances_12 = MatchFPFH(descriptors_first, descriptors_second)
    matches_21, distances_21 = MatchFPFH(descriptors_second, descriptors_first)

    matches = []
    for i in range(matches_12.shape[0]):
        idx_1 = matches_21[matches_12[i, 1], 1]
        if(idx_1 == i):
            matches.append(matches_12[i])
    return np.asarray(matches)

def show_matches(matches_12, features_first, features_second):
    lines_vis = []
    points_vis = []
    index_t = 0
    for i in range(matches_12.shape[0]):
        points_vis.append(features_first[matches_12[i,0]])
        points_vis.append(features_second[matches_12[i,1]])
        lines_vis.append([index_t , index_t+1])
        index_t = index_t + 2
    colors = [[0, 0, 0] for i in range(len(lines_vis))]

    matches_set = o3d.geometry.LineSet()
    matches_set.lines = o3d.utility.Vector2iVector(lines_vis)
    matches_set.points  = o3d.utility.Vector3dVector(points_vis)
    matches_set.colors = o3d.utility.Vector3dVector(colors)
    return matches_set

def EstimateInitialPose(features_first, features_second, matches_12):
    matched_1 = []
    matched_2 = []
    for i in range(matches_12.shape[0]):
        matched_1.append(features_first[matches_12[i,0]])
        matched_2.append(features_second[matches_12[i,1]])

    R, t = SvdBasedPoseEstimation(np.asarray(matched_1), np.asarray(matched_2))
    transformed_features = Transform_cloud(np.asarray(matched_2), R, t)
    score = np.linalg.norm(np.asarray(matched_1) - transformed_features, axis=1)

    return R, t, np.average(score)

def EstimateInitialPoseRANSAC(features_first, features_second, matches_12):
    n_iterations = 100
    n_sample = (int)(matches_12.shape[0]/5)

    R_best = 0
    t_best = 0
    score_best = 999
    for i in range(n_iterations):
        subsample_ids = np.random.choice(matches_12.shape[0], n_sample, replace=False)
        sample_matches = matches_12[subsample_ids]
        R, t, score = EstimateInitialPose(features_first, features_second, sample_matches)
        #print('iteration : ',i, ' score : ', score)
        if(score < score_best):
            R_best = R
            t_best = t
    return R_best, t_best

def FeatueMatchEstimation(cloud_first, cloud_second):
    lambda_12 = 0.25
    lambda_32 = 0.5
    features_first, _ = ISS_PFH.SSI_cloud(cloud_first,  lambda_12, lambda_32, False)
    normal_estimation_first = ISS_PFH.estimate_normal(cloud_first)
    descriptors_first = ISS_PFH.FPFH_descriptors(cloud_first, normal_estimation_first, features_first)

    features_second, _ = ISS_PFH.SSI_cloud(cloud_second,  lambda_12, lambda_32, False)
    normal_estimation_second = ISS_PFH.estimate_normal(cloud_second)
    descriptors_second = ISS_PFH.FPFH_descriptors(cloud_second, normal_estimation_second, features_second)

    matches_12 = MatchFPFH_twoway(descriptors_first, descriptors_second)
    matches_set = show_matches(matches_12, features_first, features_second)
    R, t = EstimateInitialPoseRANSAC(features_first, features_second, matches_12)
    return matches_12, R, t, matches_set
