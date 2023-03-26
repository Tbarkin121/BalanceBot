from pytorch3d.transforms import euler_angles_to_matrix, matrix_to_euler_angles, matrix_to_quaternion
from pytorch3d.transforms import  quaternion_to_matrix, quaternion_apply, quaternion_invert, quaternion_multiply
import torch


def rotationMatrixToEulerAngles(R):
        sy = torch.sqrt(R[:,0,0] * R[:,0,0] +  R[:,1,0] * R[:,1,0])
        singular = sy < 1e-6

        x_not_singular = torch.atan2(R[:,2,1] , R[:,2,2])
        y_not_singular = torch.atan2(-R[:,2,0], sy)
        z_not_singular = torch.atan2(R[:,1,0], R[:,0,0])
        x_singular = torch.atan2(-R[:,1,2], R[:,1,1])
        y_singular = torch.atan2(-R[:,2,0], sy)
        z_singular = 0
        
        x = torch.unsqueeze(torch.where(singular, x_singular, x_not_singular), dim=-1)
        y = torch.unsqueeze(torch.where(singular, y_singular, y_not_singular), dim=-1)
        z = torch.unsqueeze(torch.where(singular, z_singular, z_not_singular), dim=-1)
        return torch.cat([x, y, z], dim=-1)

def quat_abs(x):
    """
    quaternion norm (unit quaternion represents a 3D rotation, which has norm of 1)
    """
    x = x.norm(p=2, dim=-1)
    return x

def quat_unit(x):
    """
    normalized quaternion with norm of 1
    """
    norm = quat_abs(x).unsqueeze(-1)
    return x / (norm.clamp(min=1e-9))


base_quat = torch.tensor([[1.0,0.5,0.0,0.0]]).reshape(1,4).repeat(2,1)
base_quat = quat_unit(base_quat)
print(base_quat)
rot_mat = quaternion_to_matrix(base_quat)
print(rot_mat)
EA_1 = matrix_to_euler_angles(rot_mat, 'XYZ')
print(EA_1)

EA_2 = rotationMatrixToEulerAngles(rot_mat)
print(EA_2)