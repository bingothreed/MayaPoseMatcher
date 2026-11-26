# -*- coding: utf-8 -*-
"""
获取指定关节的全局位置和全局旋转。

用法:
    1. 直接运行本脚本(修改下方 JOINT_NAME 为实际关节名);
    2. 或先选中一个关节再运行,会自动使用当前选择;
    3. 或调用 get_global_info("jointName") 得到 [位置, 旋转矩阵, 欧拉角(度)]。

输出:
    - 全局位置 (MVector / list)
    - 全局旋转矩阵 (3x3 numpy 数组, 行主序, 与 om.MMatrix 一致)
    - 全局欧拉角 (XYZ, 度)
"""
import maya.cmds as cmds
import maya.api.OpenMaya as om
import numpy as np

np.set_printoptions(suppress=True, threshold=np.inf)


def get_global_info(joint_name):
    """
    获取指定关节的全局位置与全局旋转。

    参数:
        joint_name (str): 关节名

    返回:
        list: [global_position (list), global_rotation (3x3 np.ndarray),
               global_euler (list, XYZ 度)]
    """
    if not cmds.objExists(joint_name):
        raise ValueError("找不到对象: %s" % joint_name)

    # 1) 全局位置
    pos = cmds.xform(joint_name, q=True, ws=True, t=True)
    origin = om.MVector(pos)
    global_position = [origin[0], origin[1], origin[2]]

    # 2) 世界空间变换矩阵 -> 提取 3x3 旋转部分
    matrix = cmds.xform(joint_name, q=True, m=True, ws=True)
    m_matrix = om.MMatrix(matrix)
    global_rotation = np.array([
        [m_matrix[0],  m_matrix[1],  m_matrix[2]],
        [m_matrix[4],  m_matrix[5],  m_matrix[6]],
        [m_matrix[8],  m_matrix[9],  m_matrix[10]],
    ])

    # 3) 从矩阵分解出全局欧拉角(度, 旋转顺序 XYZ)
    transform = om.MTransformationMatrix(m_matrix)
    euler = transform.rotation()  # MEulerRotation
    global_euler = [
        np.degrees(euler.x),
        np.degrees(euler.y),
        np.degrees(euler.z),
    ]

    return [global_position, global_rotation, global_euler]


def print_global_info(joint_name):
    """打印指定关节的全局位置与旋转。"""
    pos, rot_mat, euler = get_global_info(joint_name)
    print("=" * 50)
    print("Joint: %s" % joint_name)
    print("Global Position : [%.6f, %.6f, %.6f]" % (pos[0], pos[1], pos[2]))
    print("Global Rotation (matrix 3x3):")
    print(rot_mat)
    print("Global Euler (XYZ, deg): [%.6f, %.6f, %.6f]" % (euler[0], euler[1], euler[2]))
    print("=" * 50)


if __name__ == "__main__":
    # 优先使用当前选择, 否则使用下方指定的关节名
    sel = cmds.ls(selection=True, type="joint")
    if sel:
        target = sel[0]
    else:
        target = "abdomenLower"  # 修改为你的关节名

    print_global_info(target)
