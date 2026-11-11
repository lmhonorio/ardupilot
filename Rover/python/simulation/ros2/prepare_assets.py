#!/usr/bin/env python3
"""Convert the local Argo assets to Harmonic without editing the ROS 1 sources."""
import argparse
import copy
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

ROVER = Path(__file__).resolve().parents[3]
ROOT = ROVER / 'ros2'
NS = '/rover_argo_N1/Instance1'
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 360


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def set_text(parent, name, value):
    child = parent.find(name)
    if child is None:
        child = ET.SubElement(parent, name)
    child.text = str(value)
    return child


def write(tree, path):
    ET.indent(tree)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


def copy_assets(source):
    for package in ('rover_argo_description', 'rover_argo_gazebo'):
        target = ROOT / 'assets' / package
        shutil.copytree(source/package/'urdf', target/'urdf', dirs_exist_ok=True)
        for path in (target/'urdf').rglob('*.xacro'):
            text = path.read_text()
            for name in ('rover_argo_description', 'rover_argo_gazebo'):
                text = text.replace('$(find '+name+')', str(ROOT/'assets'/name))
                text = text.replace('package://'+name+'/', 'file://'+str(ROOT/'assets'/name)+'/')
            path.write_text(text)
    models = ROOT/'assets/models'
    models.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source/'rover_argo_gazebo/worlds',
                    ROOT/'assets/rover_argo_gazebo/worlds', dirs_exist_ok=True)
    # Retain all existing worlds. Assets are copied locally (ignored by git).
    shutil.copytree(source/'rover_argo_gazebo/models', models, dirs_exist_ok=True)
    # Ogre2 opens Collada texture references as filenames, without URI decoding.
    for path in (ROOT/'assets').rglob('*.dae'):
        text = path.read_text()
        if '%20' in text:
            path.write_text(text.replace('%20', ' '))
    for path in models.rglob('model.sdf'):
        tree = ET.parse(path).getroot()
        for material in tree.findall('.//material'):
            script = material.find('script')
            if script is None:
                continue
            material_name = script.findtext('name', '')
            textures = sorted((path.parent/'materials/textures').glob('*'))
            images = [p for p in textures if p.suffix.lower() in ('.png','.jpg','.jpeg')]
            material.remove(script)
            if images:
                # Prefer the active material's texture, rather than any older QR image.
                for mat in (path.parent/'materials/scripts').glob('*.material'):
                    content = mat.read_text()
                    active = content.split('material '+material_name,1)[-1]
                    match = re.search(r'\btexture\s+(\S+)',active)
                    if match and (path.parent/'materials/textures'/match[1]).is_file():
                        images = [path.parent/'materials/textures'/match[1]]; break
                pbr=element(material,'pbr');metal=element(pbr,'metal')
                element(metal,'albedo_map',str(images[0]))
                element(metal,'roughness',1.0);element(metal,'metalness',0.0)
            set_text(material,'diffuse','1 1 1 1')
        write(tree,path)


def convert_robot():
    import xacro
    import collada
    import numpy as np
    xacro_file=ROOT/'assets/rover_argo_gazebo/urdf/rover_argo_N1.urdf.xacro'
    xml=xacro.process_file(str(xacro_file),mappings={
        'namespace': NS.strip('/'), 'mid360_enabled':'true','insta360_enabled':'true',
        'samsung_s23_ultra_cam_enabled':'true','hokuyo_enabled':'true'}).toxml()
    robot=ET.fromstring(xml)
    for collision in robot.findall('.//collision'):
        geometry=collision.find('geometry');mesh=geometry.find('mesh')
        if mesh is None:continue
        path=unquote(urlparse(mesh.get('filename')).path)
        asset=collada.Collada(path)
        points=np.concatenate([primitive.vertex for bound in asset.scene.objects('geometry')
                               for primitive in bound.primitives() if hasattr(primitive,'vertex')])
        points=points*float(asset.assetInfo.unitmeter or 1.)
        points*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
        low,high=points.min(axis=0),points.max(axis=0)
        origin=collision.find('origin')
        if origin is None:origin=element(collision,'origin',xyz='0 0 0',rpy='0 0 0')
        r,p,y=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
        cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
        rotation=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                           [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')+rotation@((low+high)/2)
        origin.set('xyz',' '.join(map(str,xyz)))
        geometry.remove(mesh)
        element(geometry,'box',size=' '.join(map(str,np.maximum(high-low,.0001))))
    for gz in robot.findall('gazebo'):
        for plugin in list(gz.findall('plugin')):
            if plugin.get('name')=='ArduPilotPlugin':
                plugin.set('filename','ArduPilotPlugin')
                set_text(plugin,'imuName','imu_link::imu_sensor')
                set_text(plugin,'no_time_sync',1)
                for control in plugin.findall('control'):
                    # Original file contains cmd_max=-1 < cmd_min=0: invalid PID limits.
                    set_text(control,'cmd_max',10);set_text(control,'cmd_min',-10)
            else:
                gz.remove(plugin)
        for sensor in gz.findall('sensor'):
            name=sensor.get('name')
            plugins=list(sensor.findall('plugin'))
            kind=sensor.get('type')
            if name=='mid360_sensor':
                ray=copy.deepcopy(plugins[0].find('ray'))
                sensor.set('type','gpu_lidar')
                ray.tag='lidar'
                ray.find('scan/horizontal/samples').text='720'
                ray.find('scan/horizontal/min_angle').text='-3.141592653589793'
                ray.find('scan/horizontal/max_angle').text='3.1328660073298216'
                ray.find('scan/vertical/samples').text='64'
                sensor.append(ray)
                set_text(sensor,'topic',NS+'/livox/scan')
                set_text(sensor,'gz_frame_id','mid360_sensor')
            elif name=='mid360_lio_imu':
                set_text(sensor,'topic',NS+'/livox/imu')
                set_text(sensor,'gz_frame_id','mid360_sensor')
            elif name=='imu_sensor':
                # The JSON plugin forwards raw IMU vectors in aircraft FRD.
                # It converts the MODEL pose, but does not rotate acceleration.
                set_text(sensor,'pose','0 0 0 3.141592653589793 0 0')
                # ArduPilotPlugin subscribes to the default scoped .../imu topic.
            elif kind in ('camera','wideanglecamera','wide_angle_camera'):
                is_samsung=any(p.get('name')=='s23_ros' for p in plugins)
                prefix=NS+('/camera' if is_samsung else '/camera/insta360')
                set_text(sensor,'topic',prefix+'/image_raw')
                set_text(sensor,'gz_frame_id','camera_optical' if is_samsung else 'insta360_optical')
                set_text(sensor.find('camera'),'camera_info_topic',prefix+'/camera_info')
                camera=sensor.find('camera')
                # Render at the reduced resolution; CameraInfo is generated by
                # Gazebo from this image size and the unchanged field of view.
                set_text(camera.find('image'),'width',CAMERA_WIDTH)
                set_text(camera.find('image'),'height',CAMERA_HEIGHT)
                misplaced_rate=camera.find('update_rate')
                if misplaced_rate is not None:
                    set_text(sensor,'update_rate',misplaced_rate.text);camera.remove(misplaced_rate)
                if not is_samsung:
                    sensor.set('type','wide_angle_camera')
                    lens=element(camera,'lens')
                    element(lens,'type','equidistant');element(lens,'scale_to_hfov','true')
                    element(lens,'cutoff_angle','1.5707963267948966')
                    element(lens,'env_texture_size','512')
            elif kind in ('ray','gpu_ray'):
                sensor.set('type','gpu_lidar')
                ray=sensor.find('ray')
                if ray is not None:ray.tag='lidar'
                set_text(sensor,'topic',NS+'/lidar/hokuyo/scan')
                set_text(sensor,'gz_frame_id','hokuyo')
            for plugin in plugins:sensor.remove(plugin)
            set_text(sensor,'always_on','true')
        # Keep sensor links/joints in SDF, so IMU lookup and transforms stay explicit.
    for joint in robot.findall('joint'):
        if joint.get('type')=='fixed':
            gz=element(robot,'gazebo',reference=joint.get('name'))
            element(gz,'preserveFixedJoint','true')
    gz=element(robot,'gazebo')
    jointpub=element(gz,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher')
    element(jointpub,'topic',NS+'/joint_states')
    for axis in ('pan','tilt'):
        p=element(gz,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
        element(p,'joint_name','pt_support_s23_'+axis+'_joint')
        element(p,'topic',NS+'/camera/'+axis+'_cmd')
        element(p,'p_gain',10);element(p,'i_gain',0);element(p,'d_gain',.5)
        element(p,'cmd_max',10);element(p,'cmd_min',-10)
    # Camera messages use optical coordinates (right/down/forward).
    for parent, child, xyz in [('samsung_s23_ultra_cam_link','camera_optical','0 0 0.1'),
                               ('insta360_link','insta360_optical','0.02 0 0.1')]:
        if robot.find("link[@name='%s']" % parent) is not None:
            element(robot,'link',name=child)
            joint=element(robot,'joint',name=child+'_joint',type='fixed')
            element(joint,'parent',link=parent);element(joint,'child',link=child)
            element(joint,'origin',xyz=xyz,rpy='-1.5707963267948966 0 -1.5707963267948966')
    urdf=ROOT/'generated/rover.urdf';write(robot,urdf)
    converted=subprocess.run(['gz','sdf','-p',str(urdf)],capture_output=True,text=True,check=True)
    sdf=ET.fromstring(converted.stdout)
    if sdf.find('model') is None:raise RuntimeError('Conversao URDF nao gerou model')
    sdf.find('model').set('name','rover_argo_N1')
    write(sdf,ROOT/'generated/rover.sdf')
    return sdf.find('model')


def convert_world(source, name, model):
    path=source/'rover_argo_gazebo/worlds'/(name+'.world')
    root=ET.parse(path).getroot();root.set('version','1.9')
    world=root.find('world');world.set('name',name)
    for pose in world.findall('.//pose'):
        pose.attrib.pop('frame',None)
    for uri in world.findall('include/uri'):
        if uri.text.startswith('model://'):
            asset=ROOT/'assets/models'/uri.text[len('model://'):]/'model.sdf'
            if not asset.is_file():raise FileNotFoundError(asset)
            uri.text=asset.as_uri()
    for tag in ('state','gui','plugin'):
        for item in world.findall(tag):world.remove(item)
    for name_,filename in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Sensors','sensors'),('Imu','imu')]:
        p=element(world,'plugin',filename='gz-sim-'+filename+'-system',name='gz::sim::systems::'+name_)
        if name_=='Sensors':element(p,'render_engine','ogre2')
    model=copy.deepcopy(model);set_text(model,'pose','0 0 0.155 0 0 0');world.append(model)
    write(root,ROOT/'generated/worlds'/(name+'.sdf'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path.home()/'catkin_ws/src/rover-argo-gazebo')
    p.add_argument('--world',default='gravel_plane')
    a=p.parse_args()
    if not a.source.is_dir():p.error('Repositorio de assets nao encontrado: '+str(a.source))
    copy_assets(a.source)
    model=convert_robot();convert_world(a.source,a.world,model)
    print('Assets Harmonic preparados em',ROOT/'generated')

if __name__=='__main__':main()
