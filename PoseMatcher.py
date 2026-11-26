import maya.cmds as cmds
import maya.api.OpenMaya as om
import numpy as np
import math
from typing import Tuple, List
import json
import os
import re
import gzip
import datetime

################################### Pose alignment functions Start ################################### 
# region pose alignment
def get_base_name(obj_name):
    """Get the base name of an object (excluding namespace and path)"""
    return obj_name.split(":")[-1].split("|")[-1]

def apply_global_rotation_to_vector(rotation_quaternion, vector):
    vector_mvector = om.MVector(vector[0], vector[1], vector[2])
    rotated_vector = vector_mvector.rotateBy(rotation_quaternion)
    return [rotated_vector.x, rotated_vector.y, rotated_vector.z]

def getlocalRotation(joint_name):
    return cmds.xform(joint_name, query=True, worldSpace=False, rotation=True)

def calculate_angle_3d(a, b):
    a = np.array(a)
    b = np.array(b)    
    dot_product = np.dot(a, b)    
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    cos_theta = dot_product / (norm_a * norm_b)
    theta = np.arccos(cos_theta)    
    return np.degrees(theta)

def get_joint_lra(joint, length=10.0):
    # Get the world position of the joint
    pos = cmds.xform(joint, q=True, ws=True, t=True)
    origin = om.MVector(pos)

    # 获取 jointOrient 并转换为旋转矩阵
    orient = cmds.getAttr(joint + ".jointOrient")[0]
    euler = om.MEulerRotation(
        math.radians(orient[0]),
        math.radians(orient[1]),
        math.radians(orient[2])
    )
    rot_matrix = euler.asMatrix()

    # Extract the rotation axis direction vector
    x_axis = om.MVector(rot_matrix[0], rot_matrix[1], rot_matrix[2]).normal()
    y_axis = om.MVector(rot_matrix[4], rot_matrix[5], rot_matrix[6]).normal()
    z_axis = om.MVector(rot_matrix[8], rot_matrix[9], rot_matrix[10]).normal()
    # Calculate the endpoint
    x_end = origin + x_axis * length
    y_end = origin + y_axis * length
    z_end = origin + z_axis * length
    xyzVec = []
    xyzVec.append([x_end.x-origin.x, x_end.y-origin.y, x_end.z-origin.z])
    xyzVec.append([y_end.x-origin.x, y_end.y-origin.y, y_end.z-origin.z])
    xyzVec.append([z_end.x-origin.x, z_end.y-origin.y, z_end.z-origin.z])
    return xyzVec

def get_joint_global_lra(joint, length=100.0):
    pos = cmds.xform(joint, q=True, ws=True, t=True)
    origin = om.MVector(pos)

    matrix = cmds.xform(joint, q=True, m=True, ws=True)
    m_matrix = om.MMatrix(matrix)

    joint_axis = cmds.getAttr(joint + ".rotateAxis")[0]
    order = cmds.getAttr(joint + ".rotateOrder")
    euler = om.MEulerRotation(
        math.radians(joint_axis[0]),
        math.radians(joint_axis[1]),
        math.radians(joint_axis[2]),
        order
    )
    euler_matrix = euler.asMatrix()
    rot_matrix = m_matrix*(euler_matrix.inverse())
    # Extract the rotation axis direction vector
    x_axis = om.MVector(rot_matrix[0], rot_matrix[1], rot_matrix[2]).normal()
    y_axis = om.MVector(rot_matrix[4], rot_matrix[5], rot_matrix[6]).normal()
    z_axis = om.MVector(rot_matrix[8], rot_matrix[9], rot_matrix[10]).normal()
    # Calculate the endpoint
    x_end = origin + x_axis * length
    y_end = origin + y_axis * length
    z_end = origin + z_axis * length
    xyzVec = []
    xyzVec.append([x_end.x-origin.x, x_end.y-origin.y, x_end.z-origin.z])
    xyzVec.append([y_end.x-origin.x, y_end.y-origin.y, y_end.z-origin.z])
    xyzVec.append([z_end.x-origin.x, z_end.y-origin.y, z_end.z-origin.z])
    return xyzVec

def findClose0or180(angles):
    newAngles= []
    for angle in angles:
        newAngles.append(min(angle,180-abs(angle)))
    min_value = min(newAngles)  # find the minimize value
    min_index = newAngles.index(min_value)  # find the minimize value's index
    return min_index

def getAimUpVector(joint_name, boneDir):
    lraVec = get_joint_global_lra(joint_name)
    angle_x = calculate_angle_3d(np.array(lraVec[0]),np.array(boneDir))
    angle_y = calculate_angle_3d(np.array(lraVec[1]),np.array(boneDir))
    angle_z = calculate_angle_3d(np.array(lraVec[2]),np.array(boneDir))
    axis = [[1,0,0],[0,1,0],[0,0,1]]
    angles = [angle_x, angle_y, angle_z]
    min_index = findClose0or180(angles)
    sign = 1
    if(angles[min_index]>180-abs(angles[min_index])):
        sign = -1
    
    aimVector = [sign*axis[min_index][0],sign*axis[min_index][1],sign*axis[min_index][2]]
    return [aimVector, axis[(min_index+1)%3]]

def rotate_joint_to_direction(joint_name,aimVector,target_direction):
    # create an empty object as a target
    target_object = cmds.spaceLocator(name="target")[0]
    # Position the null object at joint1's location to ensure it coincides with joint1
    cmds.delete(cmds.parentConstraint(joint_name, target_object))
    # Move the null object's position to the target direction.
    global_position = cmds.xform(joint_name, query=True, worldSpace=True, translation=True)
    cmds.move(global_position[0]+target_direction[0], global_position[1]+target_direction[1], global_position[2]+target_direction[2], target_object, absolute=True)
    # Use an aimConstraint to make joint2 point at the null object
    cmds.aimConstraint(target_object, joint_name, aimVector=aimVector)
    # if("_l" in joint_name or "l_" in joint_name):
    #     cmds.aimConstraint(target_object, joint_name, aimVector=[1,0,0],upVector=[0,1,0])
    # else:
    #     cmds.aimConstraint(target_object, joint_name, aimVector=[-1,0,0],upVector=[0,1,0])
    # Delete the temporarily created target object.
    # cmds.delete(target_object)

def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def get_joint_position(joint):
    """get joint's global position"""
    pos = cmds.xform(joint, query=True, worldSpace=True, translation=True)
    return np.array(pos)

def getGlobalRot(joint):
    matrix = cmds.xform(joint, q=True, m=True, ws=True)
    m_matrix = om.MMatrix(matrix)
    t_matrix = om.MTransformationMatrix(m_matrix)
    return t_matrix.rotation(asQuaternion=True).asMatrix()

def getJointOrient(joint):
    orient = cmds.getAttr(joint + ".jointOrient")[0]
    jo_euler = om.MEulerRotation(
        math.radians(orient[0]),
        math.radians(orient[1]),
        math.radians(orient[2])
    )
    jo_matrix = jo_euler.asMatrix()
    return jo_matrix

def getRotateAxisRot(joint):
    joint_axis = cmds.getAttr(joint + ".rotateAxis")[0]
    order = cmds.getAttr(joint + ".rotateOrder")
    ra_euler = om.MEulerRotation(
        math.radians(joint_axis[0]),
        math.radians(joint_axis[1]),
        math.radians(joint_axis[2]),
        0
    )
    ra_matrix = ra_euler.asMatrix()
    return ra_matrix

def getRotOrder(joint):
    order = cmds.getAttr(joint + ".rotateOrder")
    return order

def getAnimRotate(joint):
    animLocalRot = cmds.getAttr(joint + ".rotate")[0]
    animEuler = om.MEulerRotation(
        math.radians(animLocalRot[0]),
        math.radians(animLocalRot[1]),
        math.radians(animLocalRot[2]),
        getRotOrder(joint)
    )
    anim_matrix = animEuler.asMatrix()
    return anim_matrix

def getInherentRotWithoutParent(joint):
    jo_matrix = getJointOrient(joint)
    ra_matrix = getRotateAxisRot(joint)
    anim_matrix = getAnimRotate(joint)

    tmp_matrix=om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    result_matrix=om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    tmp_matrix.setToProduct(ra_matrix,anim_matrix)
    result_matrix.setToProduct(tmp_matrix,jo_matrix)
    return result_matrix

def get_parent_delta_global_rot(joint_name):
    inherent_global_rot = getInherentRotWithoutParent(joint_name)
    global_rot = getGlobalRot(joint_name)
    delta_rot = om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    delta_rot.setToProduct(inherent_global_rot.inverse(), global_rot)
    return delta_rot

def getNeedRot(v1:om.MVector,v2:om.MVector):
    rot_quat = v1.rotateTo(v2)
    return rot_quat.asMatrix()

def align_parent_to_vector(child_joint, target_vector):
    """Align the parent joint's direction toward its child joint with the target vector."""    
    parents = cmds.listRelatives(child_joint, parent=True, fullPath=True)
    if not parents:
        cmds.warning(f"⚠️ {get_base_name(child_joint)} has no parent to align")
        return
    parent_joint = parents[0]
    parent_base = get_base_name(parent_joint)
    if any(tag in parent_base for tag in ("twist", "Twist", "Meta", "meta")):
        next_parents = cmds.listRelatives(parent_joint, parent=True, fullPath=True)
        if not next_parents:
            cmds.warning(f"⚠️ {get_base_name(child_joint)}'s chain ends at '{parent_base}', cannot align")
            return
        parent_joint = next_parents[0]
    parent_delta_global_rot = get_parent_delta_global_rot(parent_joint)
    
    # get the position of child and parent joint
    parent_pos = get_joint_position(parent_joint)
    child_pos = get_joint_position(child_joint)
    boneVec = om.MVector(child_pos[0] - parent_pos[0], child_pos[1] - parent_pos[1],child_pos[2] - parent_pos[2])
    targetVec = om.MVector(float(target_vector[0]), float(target_vector[1]), float(target_vector[2]))
    needRot = boneVec.rotateTo(targetVec)
    needRotMat = needRot.asMatrix()

    jo_mat = getJointOrient(parent_joint)
    ra_mat = getRotateAxisRot(parent_joint)
    current_global = getGlobalRot(parent_joint)

    tmp1 = om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    tmp2 = om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    tmp3 = om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    new_rot_mat = om.MMatrix(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))
    tmp1.setToProduct(jo_mat,parent_delta_global_rot)
    tmp2.setToProduct(current_global,needRotMat)
    tmp3.setToProduct(ra_mat.inverse(),tmp2)
    new_rot_mat.setToProduct(tmp3,tmp1.inverse())

    new_rot_transMat =  om.MTransformationMatrix(new_rot_mat)
    new_rot = new_rot_transMat.rotation(asQuaternion=True)
    new_rot_euler = new_rot.asEulerRotation()

    order = getRotOrder(parent_joint)
    ordered_euler = new_rot_euler.reorderIt(order)
    # Apply the rotation to the parent joint's local rotation.
    cmds.setAttr(parent_joint + '.rotateX', math.degrees(ordered_euler[0]))
    cmds.setAttr(parent_joint + '.rotateY', math.degrees(ordered_euler[1]))
    cmds.setAttr(parent_joint + '.rotateZ', math.degrees(ordered_euler[2]))

def get_joint_name_map(root_joint):
    """Build a {base_name: full_path} dict for all joints under root_joint (one pass)."""
    all_joints = cmds.listRelatives(root_joint, allDescendents=True, type="joint", fullPath=True) or []
    all_joints.append(cmds.ls(root_joint, long=True)[0])
    name_map = {}
    for joint in all_joints:
        name_map.setdefault(get_base_name(joint), joint)
    return name_map

def find_joint_by_base_name(base_name, root_joint, name_map=None):
    """Find joints with the same base name under the specified root joint and return full path.

    Returns None when not found - the caller decides how to report it.
    """
    if name_map is None:
        name_map = get_joint_name_map(root_joint)
    return name_map.get(base_name)
    


def get_joint_direction(joint_name):
    """Get the direction vector of the joint relative to its parent joint."""
    parent = cmds.listRelatives(joint_name, parent=True, fullPath=True)
    
    if not parent:
        return [0, 0, 0]
    
    pos1 = cmds.xform(joint_name, q=True, ws=True, t=True)    
    child_trans = om.MVector(pos1)
    pos2 = cmds.xform(parent[0], q=True, ws=True, t=True)
    parent_trans = om.MVector(pos2)
    child_pos = [child_trans[0],child_trans[1],child_trans[2]]
    parent_pos = [parent_trans[0],parent_trans[1],parent_trans[2]]
    
    return [
        child_pos[0] - parent_pos[0],
        child_pos[1] - parent_pos[1],
        child_pos[2] - parent_pos[2]
    ]

def resetTwist(jointname, original_rot, rotAxis):
    rotAixsName = ".rotateX"
    rotIndex = 0
    if(rotAxis==[1,0,0]):
        rotAixsName = ".rotateX"
        rotIndex = 0
    if(rotAxis==[0,1,0]):
        rotAixsName = ".rotateY"
        rotIndex = 1
    if(rotAxis==[0,0,1]):
        rotAixsName = ".rotateZ"
        rotIndex = 2    
    cmds.setAttr(jointname + rotAixsName, original_rot[rotIndex])

def align_skeleton(joint_map, mh_root, daz_root):
    """Main function for aligning the skeleton."""
    processed_parents = set()
    error_joints = []
    mh_name_map = get_joint_name_map(mh_root)
    daz_name_map = get_joint_name_map(daz_root)

    for mh_joint_name, daz_joint_name in joint_map.items():        
        # Find MetaHuman joints.
        mh_joint = find_joint_by_base_name(mh_joint_name, mh_root, mh_name_map)
        if not mh_joint:
            cmds.warning(f"⚠️ MetaHuman joint '{mh_joint_name}' not found under root '{get_base_name(mh_root)}'")
            error_joints.append(mh_joint_name)
            continue
        
        # Find DAZ joints.
        daz_joint = find_joint_by_base_name(daz_joint_name, daz_root, daz_name_map)
        if not daz_joint:
            cmds.warning(f"⚠️ DAZ joint '{daz_joint_name}' not found under root '{get_base_name(daz_root)}'")
            error_joints.append(daz_joint_name)
            continue
        # Get the DAZ parent joint.
        daz_parents = cmds.listRelatives(daz_joint, parent=True, fullPath=True)
        if not daz_parents:
            cmds.warning(f"⚠️ DAZ joint '{get_base_name(daz_joint)}' has no parent")
            continue
        daz_parent = daz_parents[0]
        
        # Avoid processing the same parent joint multiple times.
        if daz_parent in processed_parents:
            continue
            
        # Get the direction vector.
        target_direction = get_joint_direction(mh_joint)        
        
        # Apply the rotation
        try:
            align_parent_to_vector(daz_joint,target_direction)
            processed_parents.add(daz_parent)
            cmds.warning(f"✅ Successfully aligned: {get_base_name(daz_parent)}")
        except Exception as e:
            cmds.warning(f"⚠️ Failed to align {get_base_name(daz_parent)}: {str(e)}")
            error_joints.append(mh_joint_name)
    
    # summary
    if error_joints:
        cmds.warning(f"⚠️ Alignment completed with {len(error_joints)} errors.")
    else:
        cmds.warning("🎉 Alignment completed successfully!")

def save_joint_map(file_path, joint_map):
    """Save the joint mapping to a JSON file."""
    try:
        with open(file_path, 'w') as f:
            json.dump(joint_map, f, indent=4)
        return True
    except Exception as e:
        cmds.warning(f"⚠️ Failed to save joint map: {str(e)}")
        return False

def load_joint_map(file_path):
    """Load the joint mapping from a JSON file."""
    try:
        if not os.path.exists(file_path):
            return {}
            
        with open(file_path, 'r') as f:
            data = json.load(f)
        
        # Handle compatibility with legacy formats.
        if isinstance(data, list):
            # legacy format: [{"mh_joint": "name", "daz_joint": "name"}]
            new_data = {}
            for item in data:
                if isinstance(item, dict) and "mh_joint" in item:
                    mh_name = item["mh_joint"]
                    new_data[mh_name] = item.get("daz_joint", "")
            return new_data
        
        # new format: {"mh_joint_name": "daz_joint_name"}
        return data
    except Exception as e:
        cmds.warning(f"⚠️ Failed to load joint map: {str(e)}")
        return {}

def auto_detect_namespace(joint_list):
    """Automatically detect and return the most common namespace."""
    namespaces = {}
    for joint in joint_list:
        base_name = get_base_name(joint)
        # Detect the namespace format. (mynamespace:joint)
        if ":" in joint:
            ns = joint.split(":")[0]
            namespaces[ns] = namespaces.get(ns, 0) + 1
        # Detect the prefix format (prefix_joint)
        else:
            prefix_match = re.search(r"^([^_]+)_", base_name)
            if prefix_match:
                prefix = prefix_match.group(1)
                namespaces[prefix] = namespaces.get(prefix, 0) + 1
    
    # Return the most common namespace or prefix.
    if namespaces:
        return max(namespaces, key=namespaces.get)
    return ""

_syncing_selection = False
def sync_selections(*_):
    """Synchronize the selection state between the two lists."""
    global _syncing_selection
    
    # Skip if synchronization is in progress.
    if _syncing_selection:
        return
    # Set the synchronization flag.
    _syncing_selection = True
    
    # Get the currently active list.
    active_list = cmds.textScrollList("mhJointsList", q=True, selectItem=True)
    if active_list:
        # MetaHuman list is selected
        selected_indices = cmds.textScrollList("mhJointsList", q=True, selectIndexedItem=True) or []
        if selected_indices:
            # Sync to DAZ list
            cmds.textScrollList("dazJointsList", e=True, deselectAll=True)
            for index in selected_indices:
                cmds.textScrollList("dazJointsList", e=True, selectIndexedItem=index)
    else:
        # The DAZ list is selected.
        selected_indices = cmds.textScrollList("dazJointsList", q=True, selectIndexedItem=True) or []
        if selected_indices:
            # Sync to MetaHuman list
            cmds.textScrollList("mhJointsList", e=True, deselectAll=True)
            for index in selected_indices:
                cmds.textScrollList("mhJointsList", e=True, selectIndexedItem=index)
    
    # Clear the sync flag
    _syncing_selection = False

def auto_detect_ns_cmd():
    """Automatically detect and set the DAZ namespace"""
    daz_root = cmds.textField("dazRootField", q=True, text=True)
    
    if not daz_root or not cmds.objExists(daz_root):
        cmds.warning("⚠️ Set the DAZ root joint first")
        return
    
    all_joints = cmds.listRelatives(daz_root, allDescendents=True, type="joint", fullPath=True) or []
    all_joints.append(daz_root)
    
    ns = auto_detect_namespace(all_joints)
    
    if ns:
        cmds.textField("dazNsField", e=True, text=ns)
        cmds.warning(f"🔍 Detected DAZ namespace/prefix: {ns}")
    else:
        cmds.warning("⚠️ No common DAZ namespace/prefix found")

def load_map_cmd():
    """load joint map file"""
    json_path = cmds.textField("jsonPathField", q=True, text=True)
    if not json_path or not os.path.exists(json_path):
        cmds.warning("⚠️ Select a valid JSON file path")
        return
    
    joint_map = load_joint_map(json_path)
    cmds.textScrollList("mhJointsList", e=True, removeAll=True)
    cmds.textScrollList("dazJointsList", e=True, removeAll=True)
    
    for mh_joint, daz_joint in joint_map.items():
        cmds.textScrollList("mhJointsList", e=True, append=mh_joint)
        cmds.textScrollList("dazJointsList", e=True, append=daz_joint)
    
    cmds.warning(f"📖 Loaded {len(joint_map)} mappings from {json_path}")

def save_map_cmd():
    """save joint map file"""
    json_path = cmds.textField("jsonPathField", q=True, text=True)
    if not json_path:
        cmds.warning("⚠️ Set a JSON file path first")
        return
    
    # get the map data
    joint_map = {}
    mh_joints = cmds.textScrollList("mhJointsList", q=True, allItems=True) or []
    daz_joints = cmds.textScrollList("dazJointsList", q=True, allItems=True) or []
    
    if len(mh_joints) != len(daz_joints):
        cmds.warning("⚠️ MetaHuman and DAZ joint lists must be the same length")
        return
    
    for i in range(len(mh_joints)):
        joint_map[mh_joints[i]] = daz_joints[i]
    
    # save to file
    if save_joint_map(json_path, joint_map):
        cmds.warning(f"💾 Saved {len(joint_map)} mappings to {json_path}")

def reset_table():
    """重置映射表到默认状态"""
    cmds.textScrollList("mhJointsList", e=True, removeAll=True)
    cmds.textScrollList("dazJointsList", e=True, removeAll=True)
    
    # add the default value
    defaults = [
        ("upperarm_l", "l_upperarm"),
        ("lowerarm_l", "l_forearm"),
        ("hand_l", "l_hand"),
        ("upperarm_r", "r_upperarm"),
        ("lowerarm_r", "r_forearm"),
        ("hand_r", "r_hand"),
    ]
    
    for mh, daz in defaults:
        cmds.textScrollList("mhJointsList", e=True, append=mh)
        cmds.textScrollList("dazJointsList", e=True, append=daz)
    
    cmds.warning("🔄 Reset to default mappings")

def add_selected_cmd():
    """Add the selected joint pairs to the mapping table"""
    selected = cmds.ls(selection=True, type="joint")
    if not selected or len(selected) < 2:
        cmds.warning("⚠️ Select 2 joints (MetaHuman joint first, then DAZ joint)")
        return
    
    # 获取基名称
    mh_joint = selected[0]
    daz_joint = selected[1]
    
    mh_base = get_base_name(mh_joint)
    daz_base = get_base_name(daz_joint)
    
    # 添加到列表
    cmds.textScrollList("mhJointsList", e=True, append=mh_base)
    cmds.textScrollList("dazJointsList", e=True, append=daz_base)

def remove_selected_cmd(*_):
    """Remove the selected joint pairs from the mapping table"""
    # get the select item
    mh_selected = cmds.textScrollList("mhJointsList", q=True, selectItem=True) or []
    daz_selected = cmds.textScrollList("dazJointsList", q=True, selectItem=True) or []
    
    # return if nothing select
    if not mh_selected and not daz_selected:
        return
    
    # get all the items
    mh_items = cmds.textScrollList("mhJointsList", q=True, allItems=True) or []
    daz_items = cmds.textScrollList("dazJointsList", q=True, allItems=True) or []
    
    # create the items which need to be removed
    items_to_remove = set()
    
    # add MetaHuman select item
    for item in mh_selected:
        if item in mh_items:
            items_to_remove.add(item)
    
    # add DAZ select item
    for item in daz_selected:
        if item in daz_items:
            items_to_remove.add(item)
    
    # return if nothing remove
    if not items_to_remove:
        return
    
    # remove MetaHuman items
    for item in items_to_remove:
        if item in mh_items:
            cmds.textScrollList("mhJointsList", e=True, removeItem=item)
    
    # remove DAZ items
    for item in items_to_remove:
        if item in daz_items:
            cmds.textScrollList("dazJointsList", e=True, removeItem=item)

def add_finger_mapping():
    """add fingers"""
    finger_types = ["thumb", "index", "middle", "ring", "pinky"]
    
    for side in ["_l", "_r"]:
        for finger in finger_types:
            for i in range(1, 4):
                mh_joint = f"{finger}_0{i}{side}"
                daz_joint = f"{side[1:]}_{finger}{i}"
                cmds.textScrollList("mhJointsList", e=True, append=mh_joint)
                cmds.textScrollList("dazJointsList", e=True, append=daz_joint)
    
    cmds.warning("🖐 Added finger mappings")

def auto_detect_cmd():
    """Auto detect joints"""
    # get root bone
    mh_root = cmds.textField("mhRootField", q=True, text=True)
    daz_root = cmds.textField("dazRootField", q=True, text=True)
    daz_ns = cmds.textField("dazNsField", q=True, text=True) or ""
    
    if not mh_root or not cmds.objExists(mh_root):
        cmds.warning("⚠️ Set the MetaHuman root joint first")
        return
        
    if not daz_root or not cmds.objExists(daz_root):
        cmds.warning("⚠️ Set the DAZ root joint first")
        return
    
    # reset the table
    cmds.textScrollList("mhJointsList", e=True, removeAll=True)
    cmds.textScrollList("dazJointsList", e=True, removeAll=True)
    
    # get all the metahuman joints
    mh_joints = cmds.listRelatives(mh_root, allDescendents=True, type="joint", fullPath=True) or []
    mh_joints.append(mh_root)
    
    # get all the daz joints
    daz_joints = cmds.listRelatives(daz_root, allDescendents=True, type="joint", fullPath=True) or []
    daz_joints.append(daz_root)
    
    matched_count = 0
    
    # Create Mapping - Based on Base Name Matching
    for mh_joint in mh_joints:
        mh_base = get_base_name(mh_joint)
        
        # Search for matches in DAZ joints
        for daz_joint in daz_joints:
            daz_base = get_base_name(daz_joint)
            
            # 应用命名空间
            if daz_ns and daz_base.startswith(daz_ns):
                clean_name = daz_base[len(daz_ns):]
            else:
                clean_name = daz_base
            
            if mh_base == clean_name:
                cmds.textScrollList("mhJointsList", e=True, append=mh_base)
                cmds.textScrollList("dazJointsList", e=True, append=daz_base)
                matched_count += 1
                break
    
    cmds.warning(f"🔍 Auto-detected {matched_count} joint mappings")

def execute_alignment_cmd():
    """Execute Skeleton Alignment Operation"""
    # Get Root Joint
    mh_root = cmds.textField("mhRootField", q=True, text=True)
    daz_root = cmds.textField("dazRootField", q=True, text=True)
    
    if not mh_root or not cmds.objExists(mh_root):
        cmds.warning("⚠️ Set the MetaHuman root joint first")
        return
    
    if not daz_root or not cmds.objExists(daz_root):
        cmds.warning("⚠️ Set the DAZ root joint first")
        return
    
    # Build Joint Mapping
    joint_map = {}
    mh_joints = cmds.textScrollList("mhJointsList", q=True, allItems=True) or []
    daz_joints = cmds.textScrollList("dazJointsList", q=True, allItems=True) or []
    
    if not mh_joints or not daz_joints:
        cmds.warning("⚠️ No joint mappings found")
        return
    
    if len(mh_joints) != len(daz_joints):
        cmds.warning("⚠️ MetaHuman and DAZ joint lists must be the same length")
        return
    
    for i in range(len(mh_joints)):
        joint_map[mh_joints[i]] = daz_joints[i]
    
    # Automatically save the current mapping (optional)
    # json_path = cmds.textField("jsonPathField", q=True, text=True)
    # if json_path:
    #     if save_joint_map(json_path, joint_map):
    #         cmds.warning(f"💾 Map saved to {json_path}")
    
    # Perform alignment (single undo step + exception-safe refresh)
    cmds.warning("🔧 Starting skeleton alignment...")
    cmds.undoInfo(openChunk=True, chunkName="Align Skeletons")
    try:
        cmds.refresh(suspend=True)
        align_skeleton(joint_map, mh_root, daz_root)
    except Exception as e:
        cmds.warning(f"⚠️ Alignment aborted: {str(e)}")
    finally:
        cmds.refresh(suspend=False)
        cmds.refresh()
        cmds.undoInfo(closeChunk=True)
    cmds.warning("✅ Done!")

#endregion
################################### Pose alignment functions End ################################### 

################################### Merge/Split meshes functions Start ################################### 
#region merge and split meshes
# --------------------------------------------------
# 1. 抽取单个模型的顶点 / UV / vt
# --------------------------------------------------
def get_mesh_arrays_fast(transform_path: str):
    """
    返回:
        vertices : np.ndarray (N,3)  顶点世界坐标（顶点级）
        uvs      : np.ndarray (M,2)  UV 坐标（顶点级）
        vt       : np.ndarray (K,4)  face-vertex 映射
                     [face_id, vert_id, uv_id, vn_id]
    """
    sel = om.MSelectionList()
    sel.add(transform_path)
    dag = sel.getDagPath(0)
    dag.extendToShape()
    if dag.apiType() != om.MFn.kMesh:
        raise RuntimeError(f"{transform_path} 不是多边形网格。")

    mesh = om.MFnMesh(dag)

    # 1) 顶点
    points = mesh.getPoints(om.MSpace.kWorld)
    vertices = np.array([[p.x, p.y, p.z] for p in points], dtype=np.float64)

    # 2) UV
    u_array, v_array = mesh.getUVs()
    uvs = np.column_stack((u_array, v_array)).astype(np.float64)

    # 3) VN
    normals = np.array(mesh.getVertexNormals(angleWeighted=True, space=om.MSpace.kWorld))

    # 3) 获取 face-vertex 级索引
    face_counts, vert_ids = mesh.getVertices()
    _, uv_ids   = mesh.getAssignedUVs()
    _, norm_ids = mesh.getNormalIds()

    # face_id 列
    face_ids = np.repeat(np.arange(len(face_counts), dtype=np.int32), face_counts)

    # 组装 4 列：face_id, vert_id, uv_id, vn_id
    vt = np.column_stack((face_ids, vert_ids, uv_ids, norm_ids)).astype(np.int32)

    return vertices, uvs, normals, vt


# --------------------------------------------------
# 2. 获取所有选中模型
# --------------------------------------------------
def get_selected_meshes_numpy(mode="merge") -> List[Tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    sel_list = om.MGlobal.getActiveSelectionList()
    # if sel_list.isEmpty():
    #     raise RuntimeError("请先至少选中一个多边形模型。")
    if(mode=="merge"):
        if sel_list.length() != 2:
            cmds.confirmDialog(title='Error', message='Please select only two meshes!', icon='critical')
            raise RuntimeError("Please select only two meshes!")
    elif(mode=="split"):
        if sel_list.length() != 1:
            cmds.confirmDialog(title='Error', message='Please select only one mesh!', icon='critical')
            raise RuntimeError("Please select only one mesh!")

    results = []
    for i in range(sel_list.length()):
        dag = sel_list.getDagPath(i)
        dag.extendToShape()
        if dag.apiType() != om.MFn.kMesh:
            continue
        name = sel_list.getDagPath(i).fullPathName()
        v, u, vn, vt = get_mesh_arrays_fast(name)
        results.append((name, v, u, vn, vt))

    if not results:
        raise RuntimeError("选中的节点里没有任何多边形网格。")

    return results


# --------------------------------------------------
# 3. 重合顶点检测
# --------------------------------------------------
def find_overlaps(a: np.ndarray,
                  b: np.ndarray,
                  decimals: int = 3) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    返回:
        coords : (L,3) 重合坐标
        idx_a  : (L,)  在 a 中的行索引
        idx_b  : (L,)  在 b 中的行索引
    """
    a = np.asarray(a, dtype=float).reshape(-1, 3)
    b = np.asarray(b, dtype=float).reshape(-1, 3)

    a_r = np.round(a, decimals)
    b_r = np.round(b, decimals)

    a_view = a_r.view([('', a.dtype)] * 3)
    b_view = b_r.view([('', b.dtype)] * 3)

    _, idx_a, idx_b = np.intersect1d(
        a_view, b_view, assume_unique=False, return_indices=True
    )
    return a[idx_a], idx_a, idx_b


# --------------------------------------------------
# 4. Maya 场景内选中指定顶点
# --------------------------------------------------
def select_vertices(mesh_name: str, indices: np.ndarray):
    vtx_strings = [f"{mesh_name}.vtx[{i}]" for i in indices]
    if vtx_strings:
        cmds.select(vtx_strings, replace=True)
    else:
        cmds.select(clear=True)


# --------------------------------------------------
# 5. 一键检测两个选中模型的重合顶点
# --------------------------------------------------
def detect_and_select_overlaps(name1,name2,v1,v2,decimals: int = 3):
    """
    选中两个模型 → 检测重合顶点 → 在第一个模型上选中它们
    """
    _, idx1, idx2 = find_overlaps(v1, v2, decimals=decimals)
    print(f"共找到 {len(idx1)} 个重合顶点，已在 {name1} 中选中。")
    return name1, name2, idx1, idx2

# --------------------------------------------------
# 5. 合并模型
# --------------------------------------------------
# 将顶点和面写入obj
def writeWithColor(f,v,uv,vn,IdxsWithColor,name):
    """Write v/vt/vn/f to an OBJ file (buffered for large meshes)."""
    color_set = set(IdxsWithColor) if len(IdxsWithColor) else set()
    lines = []
    for i in range(v.shape[0]):
        if(color_set):
            if i in color_set:
                lines.append("v {0} {1} {2} 1 0 0\n".format(v[i,0],v[i,1],v[i,2]))
            else:
                lines.append("v {0} {1} {2} 1 1 1\n".format(v[i,0],v[i,1],v[i,2]))
        else:
            lines.append("v {0} {1} {2}\n".format(v[i,0],v[i,1],v[i,2]))
    for i in range(uv.shape[0]):        
        lines.append("vt {0} {1}\n".format(uv[i,0],uv[i,1]))
    for i in range(vn.shape[0]):        
        lines.append("vn {0} {1} {2}\n".format(vn[i,0],vn[i,1],vn[i,2]))
    curr_fid = -1
    for i in range(f.shape[0]):
        if(f[i,0]==curr_fid):
            lines.append(" {0}/{1}/{2}".format(f[i,1]+1,f[i,2]+1,f[i,3]+1))
        else:
            lines.append(("" if curr_fid == -1 else "\n") + "f {0}/{1}/{2}".format(f[i,1]+1,f[i,2]+1,f[i,3]+1))
            curr_fid = f[i,0]
    # for i in range(len(f)):
    #     for j in range(len(f[i])):
    #         if(j==0):
    #             fp.write("f {0} ".format(f[i][j]+1))
    #         else:
    #             fp.write("{0} ".format(f[i][j]+1))
    #         if(j==len(f[i])-1):
    #             fp.write("\n")       
    with open(name, 'w', encoding='utf-8') as fp:
        fp.write(''.join(lines))

def ProcessVertices(mesh1_v, mesh2_v, overlap1, overlap2):
    """Merge mesh2's vertices into mesh1, welding the overlapping ones.

    Returns (merged_vertices, map_v) where map_v maps each mesh2 vertex
    index to its index in the merged vertex array.
    """
    n1 = mesh1_v.shape[0]
    n2 = mesh2_v.shape[0]
    map_v = np.empty(n2, dtype=np.int32)

    is_overlap = np.zeros(n2, dtype=bool)
    is_overlap[overlap2] = True

    keep_count = int(np.count_nonzero(~is_overlap))
    map_v[~is_overlap] = n1 + np.arange(keep_count, dtype=np.int32)
    # overlap1/overlap2 are paired positionally (sorted by coordinate value,
    # not by index), so fancy indexing keeps the pairing correct.
    map_v[overlap2] = overlap1

    total_v = np.concatenate([mesh1_v, mesh2_v[~is_overlap]], axis=0)
    return total_v, map_v

def is_overlap(a, b):
    """
    a, b: (x_min, y_min, x_max, y_max)
    边贴合不算重叠
    返回 True 表示两矩形严格重叠
    """
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    # 任一方向不重叠 → 整体不重叠
    no_overlap_x = ax2 <= bx1 or bx2 <= ax1
    no_overlap_y = ay2 <= by1 or by2 <= ay1

    # 反推：两个方向都不“不重叠”才重叠
    return not (no_overlap_x or no_overlap_y)

def ProcessUV(mesh1_uv, mesh2_uv):
    """Append mesh2's UVs to mesh1's, shifting mesh2's U range out of the way when ranges overlap."""
    mesh2_uv = mesh2_uv.copy()  # avoid mutating the caller's array
    uv1_range = [math.floor(min(mesh1_uv[...,0])), math.floor(min(mesh1_uv[...,1])),math.ceil(max(mesh1_uv[...,0])), math.ceil(max(mesh1_uv[...,1]))]
    uv2_range = [math.floor(min(mesh2_uv[...,0])), math.floor(min(mesh2_uv[...,1])),math.ceil(max(mesh2_uv[...,0])), math.ceil(max(mesh2_uv[...,1]))]
    if is_overlap(uv1_range, uv2_range):
        # Shift no matter which side mesh2 starts on (the old code only
        # shifted when mesh1 started left of mesh2, leaving overlaps otherwise).
        mesh2_uv[...,0] += uv1_range[3] - uv2_range[0]
    merged_uv = np.vstack([mesh1_uv, mesh2_uv])
    map_uv = np.arange(mesh2_uv.shape[0]) + mesh1_uv.shape[0]
    return merged_uv, map_uv

def ProcessVN(mesh1_vn,mesh2_vn):
    return np.vstack([mesh1_vn,mesh2_vn]), np.arange(mesh2_vn.shape[0])+mesh1_vn.shape[0]

def ProcessFaces(mesh1_f, mesh2_f, map_v, map_uv, map_vn):
    """Remap mesh2's face-vertex table into the merged index spaces and append it to mesh1's."""
    # Face ids are 0-based and contiguous, so offset mesh2's ids by mesh1's face count.
    n1_faces = int(mesh1_f[-1, 0]) + 1
    mesh2_remapped = np.column_stack((
        mesh2_f[:, 0] + n1_faces,
        map_v[mesh2_f[:, 1]],
        map_uv[mesh2_f[:, 2]],
        map_vn[mesh2_f[:, 3]],
    )).astype(np.int32)
    return np.concatenate([mesh1_f, mesh2_remapped], axis=0)

def MergeMeshes(folder_path,mesh1_f,mesh2_f,mesh1_v,mesh2_v,mesh1_uv,mesh2_uv,mesh1_vn,mesh2_vn,overlap1,overlap2):
    merged_v, map_v = ProcessVertices(mesh1_v,mesh2_v,overlap1,overlap2)
    merged_uv, map_uv = ProcessUV(mesh1_uv,mesh2_uv)
    merged_vn, map_vn = ProcessVN(mesh1_vn,mesh2_vn)
    merged_f = ProcessFaces(mesh1_f,mesh2_f,map_v,map_uv,map_vn)

    # Store merge info in .npz (much smaller/faster than JSON for large meshes).
    np.savez(
        folder_path + "_MergeInfo.npz",
        num_vertices_mesh1=mesh1_v.shape[0],
        num_vertices_mesh2=mesh2_v.shape[0],
        map_v=map_v,
        uv_mesh1=mesh1_uv,
        uv_mesh2=mesh2_uv,
        faces_mesh1=mesh1_f,
        faces_mesh2=mesh2_f,
    )

    return merged_f, merged_v,merged_uv,merged_vn

def load_merge_info(info_path):
    """Load merge info from the new .npz sidecar or a legacy .json one."""
    if info_path.lower().endswith(".npz"):
        with np.load(info_path, allow_pickle=False) as data:
            return {
                "num_vertices_mesh1": int(data["num_vertices_mesh1"]),
                "num_vertices_mesh2": int(data["num_vertices_mesh2"]),
                "map_v": data["map_v"],
                "uv_mesh1": data["uv_mesh1"],
                "uv_mesh2": data["uv_mesh2"],
                "faces_mesh1": data["faces_mesh1"],
                "faces_mesh2": data["faces_mesh2"],
            }
    # legacy JSON format
    with open(info_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)
    return {
        "num_vertices_mesh1": int(json_data["num_vertices_mesh1"]),
        "num_vertices_mesh2": int(json_data["num_vertices_mesh2"]),
        "map_v": np.array(json_data["map_v"], dtype=np.int32),
        "uv_mesh1": np.array(json_data["uv_mesh1"]),
        "uv_mesh2": np.array(json_data["uv_mesh2"]),
        "faces_mesh1": np.array(json_data["faces_mesh1"], dtype=np.int32),
        "faces_mesh2": np.array(json_data["faces_mesh2"], dtype=np.int32),
    }

def SplitMeshes(mesh_name, info_path, merged_v, merged_vn):
    info = load_merge_info(info_path)
    mesh1_v_num = info["num_vertices_mesh1"]
    mesh2_v_num = info["num_vertices_mesh2"]
    map_v = info["map_v"]
    mesh1_uv = info["uv_mesh1"]
    mesh2_uv = info["uv_mesh2"]
    mesh1_f = info["faces_mesh1"]
    mesh2_f = info["faces_mesh2"]
    base_dir = os.path.dirname(info_path)
    safe_name = mesh_name.replace(":", "_")

    # get mesh1
    split_v_mesh1 = np.array(merged_v[0:mesh1_v_num,...])
    split_vn_mesh1 = np.array(merged_vn[0:mesh1_v_num,...])
    writeWithColor(mesh1_f, split_v_mesh1, mesh1_uv, split_vn_mesh1, [], os.path.join(base_dir, safe_name + "_Part1.obj"))
    build_mesh_from_numpy(split_v_mesh1, mesh1_uv, mesh1_f, split_vn_mesh1, mesh_name + "_Part1")

    # get mesh2
    # vn was never deduplicated: mesh2's normals are the tail mesh2_v_num entries of merged_vn
    split_vn_mesh2 = np.array(merged_vn[mesh1_v_num:mesh1_v_num + mesh2_v_num, ...])
    split_v_mesh2 = np.array(merged_v[map_v, ...])
    writeWithColor(mesh2_f, split_v_mesh2, mesh2_uv, split_vn_mesh2, [], os.path.join(base_dir, safe_name + "_Part2.obj"))
    build_mesh_from_numpy(split_v_mesh2, mesh2_uv, mesh2_f, split_vn_mesh2, mesh_name + "_Part2")

def assign_lambert_to_mesh(mesh_transform, color=(0.5, 0.5, 0.5)):
    """给 transform 指定一个 Lambert 材质"""
    shape = cmds.listRelatives(mesh_transform, s=True, type='mesh')[0]
    sg = cmds.sets(renderable=True, noSurfaceShader=True, empty=True, name="lambert1SG")
    lamb = cmds.shadingNode("lambert", asShader=True, name="lambert1")
    cmds.setAttr(lamb + ".color", *color, type="double3")
    cmds.connectAttr(lamb + ".outColor", sg + ".surfaceShader", force=True)
    cmds.sets(shape, edit=True, forceElement=sg)

def build_mesh_from_numpy(v, uv, vt, vn, name='newMesh'):
    """
    v   : (N,3)  float   顶点
    uv  : (M,2)  float   顶点级 UV
    vt  : (K,4)  int     每行 [face_id, vert_id, uv_id, vn_id]
                         同一 face_id 连续出现，长度=该面顶点数
    vn  : (P,3)  float   顶点级法线
    """
    # ---------- 1. 顶点 ----------
    points = om.MPointArray([om.MPoint(*p) for p in v])

    # ---------- 2. face_counts & 所有 per-face-vertex 索引 ----------
    face_ids = vt[:, 0]
    unique, counts = np.unique(face_ids, return_counts=True)
    face_counts   = om.MIntArray(counts.tolist())
    face_connects = om.MIntArray(vt[:, 1].tolist())
    uv_ids        = om.MIntArray(vt[:, 2].tolist())
    norm_coords   = [om.MVector(*vn[i]) for i in vt[:, 3]]
    norm_arr      = om.MVectorArray(norm_coords)  # 预先生成法线数组

    # ---------- 3. 创建 mesh ----------
    mfn = om.MFnMesh()
    mobj = mfn.create(points, face_counts, face_connects)

    # ---------- 4. UV ----------
    u_arr = om.MFloatArray(uv[:, 0].tolist())
    v_arr = om.MFloatArray(uv[:, 1].tolist())
    mfn.setUVs(u_arr, v_arr)
    mfn.assignUVs(face_counts, uv_ids)

    # ---------- 5. 法线 (关键新增部分) ----------
    # norm_coords = [om.MVector(*vn[i]) for i in vt[:, 3]]  # 每行第4列是法线索引
    # norm_arr    = om.MVectorArray(norm_coords)
    # face_ids   = om.MIntArray(vt[:, 0].tolist())  # 每行第1列：面 ID
    # vertex_ids = om.MIntArray(vt[:, 1].tolist())  # 每行第2列：顶点 ID
    # mfn.setFaceVertexNormals(norm_arr, face_ids, vertex_ids, space=om.MSpace.kObject)
    
    # ---------- 6. 改名 + 默认材质 ----------
    dag   = om.MDagPath.getAPathTo(mobj)
    final = cmds.rename(dag.fullPathName(), name)
    assign_lambert_to_mesh(final)
    cmds.select(final)
    return final

# -------------------------------------------------
# 写文件（导出）
# -------------------------------------------------
# 全局变量，存一下上次选的路径，可省略
g_lastPath = cmds.workspace(q=True, rootDirectory=True)
def MergeMeshes_cmd():
    global g_lastPath

    meshes = get_selected_meshes_numpy("merge")
    filePath = cmds.fileDialog2(fileMode=0,  # 0 = 保存路径
                            startingDirectory=g_lastPath,
                            fileFilter="obj Files (*.obj);;All Files (*)")
    if filePath:
        g_lastPath = os.path.dirname(filePath[0])

        name1, v1, uv1,vn1, vt1 = meshes[0]
        name2, v2, uv2,vn2, vt2 = meshes[1]

        name1, name2, overlap_idx1, overlap_idx2 = detect_and_select_overlaps(name1,name2,v1,v2,decimals=3)
        select_vertices(name1, overlap_idx1)
        cmds.waitCursor(state=True)
        try:
            merged_f, merged_v,merged_uv,merged_vn = MergeMeshes(os.path.splitext(filePath[0])[0],vt1,vt2,v1,v2,uv1,uv2,vn1,vn2,overlap_idx1,overlap_idx2)
            filename = os.path.splitext(os.path.basename(filePath[0]))[0]
            build_mesh_from_numpy(merged_v,merged_uv,merged_f, merged_vn, name=filename)
            writeWithColor(merged_f,merged_v,merged_uv,merged_vn, [], filePath[0])
        finally:
            cmds.waitCursor(state=False)

# -------------------------------------------------
# 弹窗：让用户选 txt，然后决定读还是写
# -------------------------------------------------
def SplitMeshes_cmd():
    global g_lastPath
    path = cmds.fileDialog2(fileMode=1,  # 1 = 单选已存在文件
                            startingDirectory=g_lastPath,
                            fileFilter="Merge Info Files (*_MergeInfo.npz *.json);;All Files (*)")
    if path:
        g_lastPath = os.path.dirname(path[0])
        meshes = get_selected_meshes_numpy("split")
        meshName, merged_v, merged_uv,merged_vn, merged_vt = meshes[0]
        SplitMeshes(meshName.rsplit('|', 1)[-1],path[0], merged_v, merged_vn)
        
#endregion
################################### Merge/Split meshes functions End ###################################

################################### BVH export functions Start ###################################
# region bvh export
def _bvh_normalize(name):
    """Normalize a joint name for matching (case/underscore/namespace-insensitive)."""
    return re.sub(r"[^a-z0-9]", "", name.lower())

def parse_bvh_hierarchy(bvh_path):
    """Parse the HIERARCHY section of a BVH file into a dict tree.

    Each node: {"name", "offset", "channels", "children", "end_site"}
    """
    with open(bvh_path, "r", encoding="utf-8") as fp:
        text = fp.read().split("MOTION", 1)[0]
    lines = [l.strip() for l in text.splitlines() if l.strip()]

    def parse_block(i):
        parts = lines[i].split()
        node = {"name": parts[1] if len(parts) > 1 else None,
                "offset": None, "channels": [], "children": [], "end_site": None}
        i += 1
        if lines[i] != "{":
            raise RuntimeError("Malformed BVH hierarchy: expected '{' after node name")
        i += 1
        while lines[i] != "}":
            line = lines[i]
            if line.startswith("OFFSET"):
                node["offset"] = [float(x) for x in line.split()[1:4]]
                i += 1
            elif line.startswith("CHANNELS"):
                toks = line.split()
                node["channels"] = toks[2:2 + int(toks[1])]
                i += 1
            elif line.startswith("JOINT"):
                child, i = parse_block(i)
                node["children"].append(child)
            elif line.startswith("End Site"):
                i += 1  # consume "End Site"
                if lines[i] != "{":
                    raise RuntimeError("Malformed BVH End Site: expected '{'")
                i += 1
                node["end_site"] = [float(x) for x in lines[i].split()[1:4]]
                i += 2  # consume the OFFSET line and the closing "}"
            else:
                i += 1  # tolerate unknown lines
        return node, i + 1  # consume "}"

    start = next(i for i, l in enumerate(lines) if l.startswith("ROOT"))
    root, _ = parse_block(start)
    return root

def _maya_rot_to_bvh(joint, channel_names, bake_joint_orient=False):
    """Convert a Maya joint's local rotation to values in the given BVH channel order.

    Returns {channel_name: degrees}. The joint's rotateOrder is converted properly
    via quaternion. jointOrient is left out by default: it represents the imported
    rig's local frame, which the target tool applies itself on import.
    """
    rot = cmds.getAttr(joint + ".rotate")[0]
    order = cmds.getAttr(joint + ".rotateOrder")
    e = om.MEulerRotation(math.radians(rot[0]), math.radians(rot[1]), math.radians(rot[2]), order)
    if bake_joint_orient:
        jo = cmds.getAttr(joint + ".jointOrient")[0]
        jo_e = om.MEulerRotation(math.radians(jo[0]), math.radians(jo[1]), math.radians(jo[2]), 0)
        m = om.MMatrix()
        m.setToProduct(jo_e.asMatrix(), e.asMatrix())
        q = om.MTransformationMatrix(m).rotation(asQuaternion=True)
    else:
        q = e.asQuaternion()
    bvh_euler = q.asEulerRotation()
    order_map = {"XYZ": 0, "YZX": 1, "ZXY": 2, "XZY": 3, "YXZ": 4, "ZYX": 5}
    order_str = "".join(c[0].upper() for c in channel_names)  # e.g. "ZXY"
    if order_str in order_map:
        bvh_euler = bvh_euler.reorderIt(order_map[order_str])
    axis_map = {"Xrotation": "x", "Yrotation": "y", "Zrotation": "z"}
    return {c: math.degrees(getattr(bvh_euler, axis_map[c])) for c in channel_names}

def _build_bvh_template_from_maya(root_joint):
    """Build a BVH template tree from the Maya joint hierarchy (DAZ convention channel order)."""
    def build(joint, is_root):
        node = {
            "name": get_base_name(joint),
            "offset": [float(x) for x in cmds.getAttr(joint + ".translate")[0]],
            "channels": (["Xposition", "Yposition", "Zposition", "Zrotation", "Xrotation", "Yrotation"]
                         if is_root else ["Zrotation", "Xrotation", "Yrotation"]),
            "children": [], "end_site": None,
            "_maya": joint,
        }
        children = cmds.listRelatives(joint, children=True, type="joint", fullPath=True) or []
        if children:
            for child in children:
                node["children"].append(build(child, False))
        else:
            node["end_site"] = [0.0, 0.0, 0.0]
        return node
    return build(root_joint, True)

def _match_maya_to_template(template, root_joint):
    """Match Maya joints to template nodes by normalized name; fill node['_maya'].

    Maya joints missing from the template are appended under their matched
    template ancestor so no pose data is lost.
    """
    lookup = {}
    def index(node):
        lookup[_bvh_normalize(node["name"])] = node
        for child in node["children"]:
            index(child)
    index(template)

    def walk(maya_joint, parent_node):
        node = lookup.get(_bvh_normalize(get_base_name(maya_joint)))
        if node is None:
            node = {
                "name": get_base_name(maya_joint),
                "offset": [float(x) for x in cmds.getAttr(maya_joint + ".translate")[0]],
                "channels": ["Zrotation", "Xrotation", "Yrotation"],
                "children": [], "end_site": None,
                "_maya": maya_joint,
            }
            parent_node["children"].append(node)
        else:
            node["_maya"] = maya_joint
        children = cmds.listRelatives(maya_joint, children=True, type="joint", fullPath=True) or []
        for child in children:
            walk(child, node)

    walk(root_joint, template)

    # Appended leaves need an End Site (some importers require it).
    def fill_end_sites(node):
        if not node["children"] and node["end_site"] is None:
            node["end_site"] = [0.0, 0.0, 0.0]
        for child in node["children"]:
            fill_end_sites(child)
    fill_end_sites(template)

def _bvh_write_hierarchy(node, indent, is_root, lines):
    pad = "\t" * indent
    lines.append(f"{pad}{'ROOT' if is_root else 'JOINT'} {node['name']}")
    lines.append(pad + "{")
    off = node["offset"] or [0.0, 0.0, 0.0]
    lines.append(f"{pad}\tOFFSET {off[0]:.6f} {off[1]:.6f} {off[2]:.6f}")
    channels = node["channels"]
    lines.append(f"{pad}\tCHANNELS {len(channels)} {' '.join(channels)}")
    for child in node["children"]:
        _bvh_write_hierarchy(child, indent + 1, False, lines)
    if node["end_site"] is not None:
        es = node["end_site"]
        lines.append(f"{pad}\tEnd Site")
        lines.append(f"{pad}\t{{")
        lines.append(f"{pad}\t\tOFFSET {es[0]:.6f} {es[1]:.6f} {es[2]:.6f}")
        lines.append(f"{pad}\t}}")
    lines.append(pad + "}")

def _bvh_collect_frame(node, root_pos, bake_joint_orient, values):
    """Append this node's channel values (depth-first) to values."""
    pos_map = {"Xposition": root_pos[0], "Yposition": root_pos[1], "Zposition": root_pos[2]}
    rot_channels = [c for c in node["channels"] if c.endswith("rotation")]
    rot_map = {}
    if node.get("_maya") and rot_channels:
        rot_map = _maya_rot_to_bvh(node["_maya"], rot_channels, bake_joint_orient)
    for ch in node["channels"]:
        if ch in pos_map:
            values.append(pos_map[ch])
        elif ch in rot_map:
            values.append(rot_map[ch])
        else:
            values.append(0.0)
    for child in node["children"]:
        _bvh_collect_frame(child, root_pos, bake_joint_orient, values)

def export_daz_pose_bvh(root_joint, output_path, reference_bvh=None,
                        frame_time=1.0 / 30.0, bake_joint_orient=False):
    """Export the current pose of the skeleton under root_joint as a single-frame BVH.

    If reference_bvh is given, its HIERARCHY (joint names, offsets, End Sites,
    channel order) is used as the template so the output stays compatible with
    the tool that produced the reference (e.g. DAZ Studio). Maya joints are
    matched by normalized name (case/underscore/namespace-insensitive); template
    joints without a Maya match export zero rotation, extra Maya joints are
    appended under their matched ancestor.

    Returns the number of joints that got rotation data from Maya.
    """
    template = None
    if reference_bvh and os.path.exists(reference_bvh):
        template = parse_bvh_hierarchy(reference_bvh)
        _match_maya_to_template(template, root_joint)
    else:
        if reference_bvh:
            cmds.warning(f"⚠️ Reference BVH not found: {reference_bvh}, exporting the Maya hierarchy as-is")
        template = _build_bvh_template_from_maya(root_joint)

    root_pos = cmds.xform(root_joint, query=True, worldSpace=True, translation=True)

    lines = ["HIERARCHY"]
    _bvh_write_hierarchy(template, 0, True, lines)
    lines.append("MOTION")
    lines.append("Frames: 1")
    lines.append("Frame Time: %.7f" % frame_time)
    values = []
    _bvh_collect_frame(template, root_pos, bake_joint_orient, values)
    lines.append(" ".join("%.6f" % v for v in values))

    with open(output_path, "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines) + "\n")

    matched = 0
    def count(node):
        nonlocal matched
        if node.get("_maya"):
            matched += 1
        for child in node["children"]:
            count(child)
    count(template)
    return matched

def export_bvh_cmd():
    """Export the current DAZ pose as a BVH file."""
    daz_root = cmds.textField("dazRootField", query=True, text=True)
    out_path = cmds.textField("bvhPathField", query=True, text=True)
    ref_path = cmds.textField("bvhRefField", query=True, text=True) or None

    if not daz_root or not cmds.objExists(daz_root):
        cmds.warning("⚠️ Set the DAZ root joint first")
        return
    if not out_path:
        cmds.warning("⚠️ Set an output BVH file path first")
        return
    if not out_path.lower().endswith(".bvh"):
        out_path += ".bvh"
        cmds.textField("bvhPathField", edit=True, text=out_path)

    try:
        matched = export_daz_pose_bvh(daz_root, out_path, reference_bvh=ref_path)
        cmds.warning(f"📦 Exported pose BVH: {out_path} ({matched} joints with pose data)")
    except Exception as e:
        cmds.warning(f"⚠️ BVH export failed: {str(e)}")

#endregion
################################### BVH export functions End ###################################

################################### DUF export functions Start ###################################
# region duf export
def export_daz_pose_duf(root_joint, output_path, bake_joint_orient=False):
    """Export the current pose of the DAZ skeleton under root_joint as a DAZ Studio pose preset (.duf).

    Writes the compressed-JSON pose preset format DAZ Studio saves (verified
    against real G9 preset files). Joint names are used as-is: DAZ G8/G9 bone
    labels match the BVH/FBX names, and the 'name://@selection/' urls make the
    preset figure-agnostic (DAZ matches the selected figure's bones by name).

    Rotation values are passed through from Maya .rotate - DAZ rotation channels
    are in degrees and share each bone's rotation order, so values map 1:1 when
    the scene was imported from DAZ FBX. Translation is written as 0 and scale as
    1 (DAZ pose defaults): Maya .translate holds the bone offsets, NOT pose
    translations, and must not be exported.

    Returns the number of joints exported.
    """
    joints = [root_joint] + (cmds.listRelatives(root_joint, allDescendents=True, type="joint", fullPath=True) or [])

    animations = []
    for j in joints:
        name = get_base_name(j)
        rot = cmds.getAttr(j + ".rotate")[0]
        if bake_joint_orient:
            # Compose the imported rest orientation into the rotation
            # (use when the exported pose drifts on re-import).
            jo = cmds.getAttr(j + ".jointOrient")[0]
            order = cmds.getAttr(j + ".rotateOrder")
            r = om.MEulerRotation(math.radians(rot[0]), math.radians(rot[1]), math.radians(rot[2]), order)
            jr = om.MEulerRotation(math.radians(jo[0]), math.radians(jo[1]), math.radians(jo[2]), 0)
            m = om.MMatrix()
            m.setToProduct(jr.asMatrix(), r.asMatrix())
            e = om.MTransformationMatrix(m).rotation(asQuaternion=True).asEulerRotation()
            e = e.reorderIt(order)
            rot = (math.degrees(e.x), math.degrees(e.y), math.degrees(e.z))

        entries = [("translation/x", 0.0), ("translation/y", 0.0), ("translation/z", 0.0),
                   ("rotation/x", rot[0]), ("rotation/y", rot[1]), ("rotation/z", rot[2]),
                   ("scale/general", 1.0), ("scale/x", 1.0), ("scale/y", 1.0), ("scale/z", 1.0)]
        for prop, value in entries:
            value = 0.0 if abs(value) < 1e-12 else round(float(value), 6)
            animations.append({
                "url": f"name://@selection/{name}:?{prop}/value",
                "keys": [[0, value]],
            })

    doc = {
        "file_version": "0.6.2.0",
        "asset_info": {
            "id": "/Presets/Poses/" + os.path.basename(output_path),
            "type": "preset_pose",
            "contributor": {"author": "", "email": "", "website": ""},
            "revision": "1.0",
            "modified": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        },
        "scene": {"animations": animations},
    }
    data = json.dumps(doc, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    with gzip.GzipFile(filename=output_path, mode="wb", compresslevel=9, mtime=0) as fp:
        fp.write(data)
    return len(joints)

def export_duf_cmd():
    """Export the current DAZ pose as a DAZ Studio pose preset (.duf)."""
    daz_root = cmds.textField("dazRootField", query=True, text=True)
    out_path = cmds.textField("dufPathField", query=True, text=True)

    if not daz_root or not cmds.objExists(daz_root):
        cmds.warning("⚠️ Set the DAZ root joint first")
        return
    if not out_path:
        cmds.warning("⚠️ Set an output DUF file path first")
        return
    if not out_path.lower().endswith(".duf"):
        out_path += ".duf"
        cmds.textField("dufPathField", edit=True, text=out_path)

    try:
        count = export_daz_pose_duf(daz_root, out_path)
        cmds.warning(f"📦 Exported pose DUF: {out_path} ({count} joints)")
    except Exception as e:
        cmds.warning(f"⚠️ DUF export failed: {str(e)}")

#endregion
################################### DUF export functions End ###################################

def create_alignment_ui():
    """Create the UI window"""
    win_name = "skeletonAlignmentUI"
    if cmds.window(win_name, exists=True):
        cmds.deleteUI(win_name)
    
    # Create main window - Fixed size to ensure all content is visible
    cmds.window(win_name, title="Skeleton Alignment Tool", width=520, height=880, sizeable=False)

    # Main layout - Vertical column layout
    main_layout = cmds.columnLayout(
        adjustableColumn=True,
        columnAttach=('both', 5),
        rowSpacing=10,
        height=870
    )
    
    # ================ set skeleton ================
    skeleton_frame = cmds.frameLayout(
        label="Skeleton Settings",
        collapsable=True,
        borderStyle="etchedIn",
        marginWidth=5,
        marginHeight=5
    )
    
    # MetaHuman root joint
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="MetaHuman Root:", align="right")
    mh_root_field = cmds.textField("mhRootField", width=120)
    cmds.button(
        label="Get Selected", 
        annotation="Select MetaHuman root joint and click",
        command=lambda *_: cmds.textField(
            mh_root_field, 
            e=True, 
            text=cmds.ls(selection=True)[0] if cmds.ls(selection=True) else ""
        )
    )
    cmds.setParent("..")
    
    # DAZ root joint
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="DAZ Root Joint:", align="right")
    daz_root_field = cmds.textField("dazRootField", width=120)
    cmds.button(
        label="Get Selected", 
        annotation="Select DAZ root joint and click",
        command=lambda *_: cmds.textField(
            daz_root_field, 
            e=True, 
            text=cmds.ls(selection=True)[0] if cmds.ls(selection=True) else ""
        )
    )
    cmds.setParent("..")
    
    # DAZ namepsace
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="DAZ Namespace:", align="right")
    daz_ns_field = cmds.textField("dazNsField", width=120)
    cmds.button(
        label="Auto Detect", 
        command=lambda *_: auto_detect_ns_cmd()
    )
    cmds.setParent("..")
    cmds.setParent("..")
    
    # ================ file ================
    file_frame = cmds.frameLayout(
        label="File Settings",
        collapsable=True,
        borderStyle="etchedIn",
        marginWidth=5,
        marginHeight=5
    )
    
    # JSON file
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="JSON File Path:", align="right")
    json_path_field = cmds.textField("jsonPathField", width=120)
    cmds.button(
        label="Browse...", 
        command=lambda *_: cmds.textField(
            json_path_field, 
            e=True, 
            text=(cmds.fileDialog2(
                fileMode=1,
                fileFilter="JSON Files (*.json)"
            ) or [""])[0]
        )
    )
    cmds.setParent("..")
    
    # file button
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.button(label="Load Map", command=lambda *_: load_map_cmd(), height=30)
    cmds.button(label="Save Map", command=lambda *_: save_map_cmd(), height=30)
    cmds.button(label="Reset Map", command=reset_table, height=30)
    cmds.setParent("..")
    cmds.setParent("..")
    
    # ================ joint map ================
    map_frame = cmds.frameLayout(
        label="Joint Mapping",
        collapsable=False,
        borderStyle="etchedIn",
        marginWidth=5,
        marginHeight=5
    )
    
    # row title
    cmds.gridLayout(numberOfColumns=2, cellWidth=250)
    cmds.text(label="MetaHuman Joints", align="center", height=20)
    cmds.text(label="Your Model Joints", align="center", height=20)
    cmds.setParent("..")
    
    # List area - Ensures sufficient height
    cmds.rowLayout(numberOfColumns=2, height=200)
    mh_list = cmds.textScrollList("mhJointsList", allowMultiSelection=True, height=200, width=250, selectCommand=sync_selections)
    daz_list = cmds.textScrollList("dazJointsList", allowMultiSelection=True, height=200, width=250, selectCommand=sync_selections)
    cmds.setParent("..")
    
    # Controll button
    cmds.gridLayout(numberOfColumns=4, cellWidth=130, cellHeight=30)
    cmds.button(label="Add Selected", command=lambda *_:add_selected_cmd())
    cmds.button(label="Remove Selected", command=lambda *_:remove_selected_cmd())
    cmds.button(label="Auto-Detect All", command=lambda *_:auto_detect_cmd())
    cmds.button(label="Add Fingers", command=lambda *_:add_finger_mapping())
    cmds.setParent("..")
    cmds.setParent("..")
    
    # ================ Execute ================
    cmds.separator(height=10)
    cmds.button(
        label="ALIGN SKELETONS", 
        height=50, 
        backgroundColor=[0.3, 0.6, 0.8],
        command=lambda *_: execute_alignment_cmd()
    )

    # ================ Mesh tool ================    
    mesh_frame = cmds.frameLayout(
        label="Mesh utils",
        collapsable=True,
        borderStyle="etchedIn",
        marginWidth=5,
        marginHeight=5
    )
    cmds.gridLayout(numberOfColumns=2, cellWidth=170)
    cmds.button(label="Merge to one mesh", command=lambda *_: MergeMeshes_cmd(), height=30)
    cmds.button(label="Restore to two meshes", command=lambda *_: SplitMeshes_cmd(), height=30)
    cmds.setParent("..")
    cmds.setParent("..")

    # ================ BVH export ================
    bvh_frame = cmds.frameLayout(
        label="BVH Export",
        collapsable=True,
        borderStyle="etchedIn",
        marginWidth=5,
        marginHeight=5
    )
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="Output BVH:", align="right")
    bvh_path_field = cmds.textField("bvhPathField", width=120)
    cmds.button(
        label="Browse...",
        annotation="Where to save the exported pose BVH",
        command=lambda *_: cmds.textField(
            bvh_path_field,
            e=True,
            text=(cmds.fileDialog2(
                fileMode=0,
                fileFilter="BVH Files (*.bvh)"
            ) or [""])[0]
        )
    )
    cmds.setParent("..")

    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="Reference BVH:", align="right")
    bvh_ref_field = cmds.textField("bvhRefField", width=120)
    cmds.button(
        label="Browse...",
        annotation="Optional: a BVH from DAZ for the same character. Its hierarchy/names/channel order are used as the export template so the result re-imports cleanly.",
        command=lambda *_: cmds.textField(
            bvh_ref_field,
            e=True,
            text=(cmds.fileDialog2(
                fileMode=1,
                fileFilter="BVH Files (*.bvh)"
            ) or [""])[0]
        )
    )
    cmds.setParent("..")

    cmds.gridLayout(numberOfColumns=1, cellWidth=500)
    cmds.button(label="Export DAZ Pose as BVH", command=lambda *_: export_bvh_cmd(), height=30)
    cmds.setParent("..")
    cmds.setParent("..")

    # ================ DUF export ================
    duf_frame = cmds.frameLayout(
        label="DUF Export (DAZ Pose Preset)",
        collapsable=True,
        borderStyle="etchedIn",
        marginWidth=5,
        marginHeight=5
    )
    cmds.gridLayout(numberOfColumns=3, cellWidth=170)
    cmds.text(label="Output DUF:", align="right")
    duf_path_field = cmds.textField("dufPathField", width=120)
    cmds.button(
        label="Browse...",
        annotation="Where to save the pose preset. Put it in a DAZ content folder (e.g. .../My Library/Presets/Poses) to see it in DAZ Studio.",
        command=lambda *_: cmds.textField(
            duf_path_field,
            e=True,
            text=(cmds.fileDialog2(
                fileMode=0,
                fileFilter="DAZ Pose Preset (*.duf)"
            ) or [""])[0]
        )
    )
    cmds.setParent("..")

    cmds.gridLayout(numberOfColumns=1, cellWidth=500)
    cmds.button(label="Export DAZ Pose as DUF", command=lambda *_: export_duf_cmd(), height=30)
    cmds.setParent("..")
    cmds.setParent("..")

    # initialize
    reset_table()
    
    cmds.showWindow(win_name)

# Start the UI
if __name__ == "__main__":
    create_alignment_ui()