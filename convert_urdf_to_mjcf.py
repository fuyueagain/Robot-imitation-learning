import mujoco
import os

# ========== 修改这里 ==========
# 路径写法（跨 Windows / Linux）：
#   普通字符串里写 Windows 反斜杠路径有转义陷阱：'\a' 会变响铃符、'\t' 变制表符等，
#   例如 'D:\...\assets\...' 中的 \a 会把路径破坏成 '...\x07ssets\...'。
#   推荐用正斜杠（Windows 与 Linux 通用，open()/os.path/mujoco 均支持）；
#   若习惯反斜杠，请写成原始字符串 r'D:\...' 或把每个 \ 写成 \\。
urdf_path = 'D:/Robot-imitation-learning/assets/LingLong2.0/LingLong2.0.urdf'
out_path = 'D:/Robot-imitation-learning/assets/LingLong2.0/LingLong2.0.xml'
# =============================

# 1. 读取原始 URDF
with open(urdf_path, 'r') as f:
    urdf_text = f.read()

# 2. 去掉 meshdir 指令，避免 meshes/meshes/ 双前缀
urdf_text = urdf_text.replace('meshdir="meshes/"', '')

# 3. 写临时文件
urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
tmp_path = os.path.join(urdf_dir, '_tmp_LingLong2.urdf')
with open(tmp_path, 'w') as f:
    f.write(urdf_text)

# 4. MuJoCo 解析并保存 MJCF
try:
    model = mujoco.MjModel.from_xml_path(tmp_path)
finally:
    os.remove(tmp_path)  # 临时文件用完自动删除

mujoco.mj_saveLastXML(out_path, model)

# 5. URDF 导入默认不生成 actuator（nu 为 0），为每个非固定关节自动添加 motor
actuator_lines = []
_fixed_jnt = getattr(mujoco.mjtJoint, "mjJNT_FIXED", None)  # mujoco>=3 已移除 mjJNT_FIXED：fixed 关节编译时自动折叠，不会出现在模型里
for j in range(model.njnt):
    if _fixed_jnt is None or model.jnt_type[j] != _fixed_jnt:
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jname:
            actuator_lines.append(f'    <motor joint="{jname}"/>')

if actuator_lines:
    actuator_xml = '<actuator>\n' + '\n'.join(actuator_lines) + '\n</actuator>\n'
    with open(out_path, 'r', encoding='utf-8') as f:
        xml_text = f.read()
    if '<actuator>' not in xml_text:
        xml_text = xml_text.replace('</mujoco>', actuator_xml + '</mujoco>')
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write(xml_text)
        print(f'✅ 已为 {len(actuator_lines)} 个运动关节添加 motor actuator')
    else:
        print('⚠️ MJCF 已存在 actuator，跳过添加')

# 6. 补浮动基座（指南 2.2 ①）
#    URDF 里 base_link 是通过 fixed joint(world_base_joint) 连到 world 的，
#    mujoco>=3 编译时会折叠 fixed 关节，导致 MJCF 中没有 <body name="base_link">，
#    只剩它的 mesh geom 挂在 <worldbody> 下。这里自动把 base 提升为浮动基座：
#      <body name="base_link" pos="0 0 0.9"><freejoint name="base_free"/></body>
import xml.etree.ElementTree as ET

tree = ET.parse(out_path)
root = tree.getroot()
worldbody = root.find('worldbody')
if worldbody is not None:
    base_geoms = [c for c in list(worldbody) if c.tag == 'geom' and c.get('mesh') == 'base_link']
    robot_bodies = [c for c in list(worldbody) if c.tag == 'body']
    if robot_bodies:
        base_body = ET.Element('body', {'name': 'base_link', 'pos': '0 0 0.9'})
        ET.SubElement(base_body, 'freejoint', {'name': 'base_free'})
        for g in base_geoms:
            worldbody.remove(g)
            base_body.append(g)
        for b in robot_bodies:
            worldbody.remove(b)
            base_body.append(b)
        worldbody.insert(0, base_body)
        ET.indent(tree, space='  ')
        tree.write(out_path, encoding='utf-8', xml_declaration=True)
        print('✅ 已为 base_link 补浮动基座 freejoint（nq 将 +7）')
    else:
        print('⚠️ worldbody 下没有 body，跳过补浮动基座')
else:
    print('⚠️ 未找到 worldbody，跳过补浮动基座')

# 7. 补 yaw 关节轴（指南 2.2 ②）
#    URDF 中部分 yaw 关节没有 axis，转换后仍缺省；这里给常见 yaw 关节补 axis="0 0 1"
#    （负向前瞻保证已有 axis 的关节不会被重复添加）。
import re
_yaw_targets = r'waist_yaw_joint|head_yaw_joint|left_shoulder_yaw_joint|right_shoulder_yaw_joint|left_wrist_yaw_joint|right_wrist_yaw_joint'
with open(out_path, 'r', encoding='utf-8') as f:
    _xml_text = f.read()
_xml_text, _n_subs = re.subn(
    r'(<joint name="(?:' + _yaw_targets + r')"(?![^>]*axis=)[^>]*?)( />)',
    r'\1 axis="0 0 1"\2',
    _xml_text,
)
with open(out_path, 'w', encoding='utf-8') as f:
    f.write(_xml_text)
print(f'✅ 已为 {_n_subs} 个缺 axis 的 yaw 关节补 axis="0 0 1"')

# 8. 重新加载确认
model = mujoco.MjModel.from_xml_path(out_path)
print('✅ 转换成功！nq=', model.nq, 'nu=', model.nu)
