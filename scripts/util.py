import math
import time
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovarianceStamped
import numpy as np

class MapToRealConverter:
    def __init__(self, grid_width=602, grid_height=602, scale_factor=0.05):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.scale_factor = scale_factor

    def OccupancyGridToReal(self, pixel_nb):
        x_grid = pixel_nb % self.grid_width
        y_grid = int(pixel_nb/self.grid_height)
        x = x_grid * self.scale_factor
        y = y_grid * self.scale_factor

        return x,y

    def RealToOccupancyGrid(self, x, y):
        x_grid = int(x / self.scale_factor)
        y_grid = int(y / self.scale_factor)
        pixel_nb = x_grid + y_grid*self.grid_height

        return pixel_nb


def covMatrix6to3(matrix):
    row1 = [matrix[0], matrix[1], matrix[5]]
    row2 = [matrix[6], matrix[7], matrix[11]]
    row3 = [matrix[30], matrix[31], matrix[35]]

    return np.array([row1,row2,row3])

def covMatrix3to6(matrix):
    final = np.zeros(36)
    for i in range(2):
        final[i*6] = matrix[i,0]
        final[1+i*6] = matrix[i,1]
        final[i*6+5] = matrix[i,2]

    final[30] = matrix[2,0]
    final[31] = matrix[2,1]
    final[35] = matrix[2,2]
    return np.array(final)

def pose_array_to_matrix(poses):
    matrix = []
    for pose in poses:
        matrix.append(np.array(pose))
    return np.array(matrix)

def vectorToPose(vector):
    pose = Pose()
    pose.position.x = vector[0]
    pose.position.y = vector[1]
    pose.orientation = rotateQuaternion(Quaternion(w=1), vector[2])
    return pose

def gaussianFromWeightedParticules(poses, weight):
    quaternion = Quaternion(w=1)
    poses_matrix = pose_array_to_matrix(poses)

    mean = np.average(poses_matrix, axis=0, weights=weight)
    Q =  np.divide(np.matmul(np.transpose(poses_matrix-mean),(poses_matrix-mean)),len(poses)-1)

    mean_pose = vectorToPose(mean)


    result_pose = PoseWithCovarianceStamped()
    result_pose.pose.pose = mean_pose
    result_pose.pose.covariance = covMatrix3to6(Q)

    return result_pose


def multiplyGaussianDistributions(A,B):
    cov_A = covMatrix6to3(np.array(A.covariance))
    cov_B = covMatrix6to3(np.array(B.covariance))
    mu_a = np.array([A.pose.position.x, A.pose.position.y, getHeading(A.pose.orientation)])
    mu_b = np.array([B.pose.position.x, B.pose.position.y, getHeading(B.pose.orientation)])
    covAplusB = cov_A + cov_B

    invCovA_CovB = np.linalg.inv(covAplusB)
    cov_C = np.matmul(np.matmul(cov_A, invCovA_CovB), cov_B)
    #cov_c = np.linalg.inv(np.linalg.inv(cov_A)+ np.linalg.inv(cov_B))
    mu_C = np.matmul(np.matmul(cov_B, invCovA_CovB), mu_a) + np.matmul(np.matmul(cov_A, invCovA_CovB), mu_b)
    pose_cov =  PoseWithCovarianceStamped()
    pose_cov.pose.pose.position.x = mu_C[0]
    pose_cov.pose.pose.position.y = mu_C[1]
    pose_cov.pose.pose.position.z = 0

    pose_cov.pose.pose.orientation = rotateQuaternion(Quaternion(w=1), mu_C[2])
    pose_cov.pose.covariance = covMatrix3to6(cov_C)
    return pose_cov

def timed(fn):
    """ Decorator to time functions. For debugging time critical code """
    def timed(*args,  **kwargs):
        t = time.time()
        print ("[", fn, __name__, "]Start: ",  t)
        ret =  fn(*args, **kwargs)
        print ("[", fn, __name__, "]End:", time.time(),  " = = = ",  time.time() - t)
        return ret
    return timed

def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.

    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0

    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)

    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # ----- Multiply new (heading-only) quaternion by the existing (pitch and bank)
    # ----- quaternion. Order is important! Original orientation is the second
    # ----- argument rotation which will be applied to the quaternion is the first
    # ----- argument.
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.

    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()

    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.

    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw
