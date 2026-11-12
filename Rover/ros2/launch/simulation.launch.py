"""Harmonic world, sensor bridges and measured transforms; no arming/missions."""
import copy
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]
DEFAULTS={'world':'gravel_plane','gui':'true','N':'1','start_x':'0','start_y':'0','z':'0.155',
          'dx':'3','dy':'3','mid360_enabled':'true','insta360_enabled':'false',
          'samsung_s23_ultra_cam_enabled':'true','hokuyo_enabled':'true',
          'headless_rendering':'false','real_time_factor':'1.0'}


def start(context):
    values={k:LaunchConfiguration(k).perform(context) for k in DEFAULTS}
    count=int(values['N'])
    if not 1<=count<=10:raise ValueError('N deve estar entre 1 e 10')
    x,y,z,dx,dy=[float(values[k]) for k in ('start_x','start_y','z','dx','dy')]
    if not all(math.isfinite(v) for v in (x,y,z,dx,dy)):raise ValueError('Posicao inicial invalida')
    path=ROOT/'generated/worlds'/(values['world']+'.sdf')
    if not path.is_file():raise RuntimeError('Prepare o mundo: python/simulation/ros2/prepare_assets.py --world '+values['world'])
    root=ET.parse(path).getroot();world=root.find('world')
    rtf=float(values['real_time_factor'])
    if not math.isfinite(rtf) or rtf<=0:raise ValueError('real_time_factor deve ser positivo')
    physics=world.find('physics')
    if physics is None:physics=ET.SubElement(world,'physics',{'name':'physics','type':'ignored'})
    factor=physics.find('real_time_factor')
    if factor is None:factor=ET.SubElement(physics,'real_time_factor')
    factor.text=str(rtf)
    original=world.find("model[@name='rover_argo_N1']")
    world.remove(original)
    template=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
    bridges=[template[0]];nodes=[]
    urdf=(ROOT/'generated/rover.urdf').read_text()
    for i in range(count):
        ns='/rover_argo_N1/Instance%d'%(i+1)
        prefix='' if i==0 else 'rover_%d/'%(i+1)
        model=copy.deepcopy(original);model.set('name','rover_argo_N%d'%(i+1))
        model.find('pose').text='%g %g %g 0 0 0'%(x+i*dx,y+i*dy,max(z,.155))
        model.find("plugin[@name='ArduPilotPlugin']/fdm_port_in").text=str(9002+10*i)
        for parent in model.iter():
            for sensor in list(parent.findall('sensor')):
                name=sensor.get('name')
                flag=('mid360_enabled' if name.startswith('mid360') else
                      'samsung_s23_ultra_cam_enabled' if name=='combined_camera' else
                      'insta360_enabled' if name.startswith('insta360') else
                      'hokuyo_enabled' if name.startswith('hokuyo') else None)
                if flag and values[flag].lower()!='true':parent.remove(sensor)
        for node in model.iter():
            if node.text:
                node.text=node.text.replace('/rover_argo_N1/Instance1',ns)
                if node.tag=='gz_frame_id':node.text=prefix+node.text
        world.append(model)
        for bridge in template[1:]:
            entry=copy.deepcopy(bridge)
            entry['ros_topic_name']=entry['ros_topic_name'].replace('/rover_argo_N1/Instance1',ns)
            entry['gz_topic_name']=entry['gz_topic_name'].replace('/rover_argo_N1/Instance1',ns)
            bridges.append(entry)
        nodes.extend([
            Node(package='robot_state_publisher',executable='robot_state_publisher',namespace=ns,
                 parameters=[{'robot_description':urdf,'use_sim_time':True,'frame_prefix':prefix}],
                 remappings=[('joint_states',ns+'/joint_states')],output='screen'),
            Node(package='tf2_ros',executable='static_transform_publisher',name='mid360_frame_%d'%i,
                 arguments=['--x','-0.003','--y','0','--z','0.434','--frame-id',prefix+'base_link',
                            '--child-frame-id',prefix+'mid360_sensor'],parameters=[{'use_sim_time':True}])])
        for name, parent, child in (
            ('hokuyo',prefix+'Instancia2_hokuyo_lidar_link',prefix+'hokuyo'),
            # Harmonic's wide-angle sensor currently emits its scoped frame name.
            ('insta360',prefix+'insta360_optical',model.get('name')+'/insta360_link/insta360_x3_sensor')):
            nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
                name='%s_alias_%d'%(name,i),arguments=['--frame-id',parent,'--child-frame-id',child],
                parameters=[{'use_sim_time':True}]))
    run=ROOT/'run/launch'/str(os.getpid());run.mkdir(parents=True,exist_ok=True)
    scene=run/'world.sdf';ET.ElementTree(root).write(scene,encoding='utf-8',xml_declaration=True)
    bridge_file=run/'bridge.yaml';bridge_file.write_text(yaml.safe_dump(bridges))
    # Large image/cloud writes must not queue /clock or IMU callbacks. Keep
    # independent transport processes and only the latest large sensor frame.
    nodes.append(ExecuteProcess(cmd=[str(ROOT/'venv/bin/python'),
        str(ROOT.parent/'python/simulation/ros2/clock_bridge.py'),'--hz','50'],output='screen'))
    groups={'time_imu':[],'cloud':[],'visual':[]}
    for entry in bridges:
        if entry['gz_type_name']=='gz.msgs.Clock':continue
        name=('time_imu' if entry['gz_type_name']=='gz.msgs.IMU'
              else 'cloud' if entry['gz_type_name']=='gz.msgs.PointCloudPacked' else 'visual')
        item=copy.deepcopy(entry)
        if name=='cloud':
            item.update(publisher_queue=1,subscriber_queue=1)
            item.pop('qos_profile',None)  # rclcpp default: RELIABLE, volatile
        groups[name].append(item)
    for name,entries in groups.items():
        config=run/('bridge_'+name+'.yaml');config.write_text(yaml.safe_dump(entries))
        env={'RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS'}
        if name!='time_imu':env['GZ_TRANSPORT_RCVHWM']='2'
        nodes.append(Node(package='ros_gz_bridge',executable='parameter_bridge',
            name='rover_'+name+'_bridge',parameters=[{'config_file':str(config),'use_sim_time':True}],
            additional_env=env,output='screen'))
    # Server-only and EGL rendering are different choices. On this GPU the
    # EGL path silently loses cubemap sectors in a 360-degree GPU lidar.
    # Use the normal display renderer even without the Gazebo GUI by default.
    server=[] if values['gui'].lower()=='true' else ['-s']
    egl=values['headless_rendering'].lower()=='true'
    if not egl and not os.environ.get('DISPLAY'):
        raise RuntimeError('Renderizacao requer DISPLAY (ou Xvfb). EGL explicito: headless_rendering:=true; valide todos os setores do LiDAR antes de navegar.')
    gazebo=ExecuteProcess(cmd=['gz','sim','-r']+server+(['--headless-rendering'] if egl else [])+[str(scene)],output='screen')
    return [gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo encerrou'))]))]+nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in DEFAULTS.items()]+[OpaqueFunction(function=start)])
