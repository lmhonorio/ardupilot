#!/usr/bin/python3.8
"""Build the official GLIM CPU/ROS1 stack in Rover/python/.deps/glim, without sudo."""
import argparse
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parents[2] / '.deps/glim'
PREFIX = ROOT / 'install'
SOURCES = {
    'gtsam': ('borglab/gtsam', 'c57988fe554e7213c77fe379c1d7c483de26ad33'),  # 4.2a9
    'gtsam_points': ('koide3/gtsam_points', 'dff0518ae456537a2b0b1362213e32952574a98e'),
    'glim': ('koide3/glim', '65a5562c6b8906e60693c63cc0c95de885426352'),  # 1.1.0
    'glim_ros1': ('koide3/glim_ros1', '84e203f19d2f5d41890a02b258eac6c0047f339a'),
    'spdlog': ('gabime/spdlog', '76fb40d95455f249bd70824ecfcae7a8f0930fa3'),  # 1.10.0
    'fmt': ('fmtlib/fmt', 'b6f4ceaed0a0a24ccf575fab6c56dd50ccf6f1a9'),  # 8.1.1
}


def fetch(name):
    target = ROOT / 'src' / name
    if target.is_dir():
        return target
    repo, revision = SOURCES[name]
    url = 'https://codeload.github.com/%s/tar.gz/%s' % (repo, revision)
    data = urllib.request.urlopen(url, timeout=120).read()
    target.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        members = archive.getmembers()
        for member in members:
            path = (target.parent / member.name).resolve()
            if target.parent.resolve() not in path.parents or member.issym() or member.islnk():
                raise RuntimeError('Unsafe source archive member: ' + member.name)
        archive.extractall(target.parent)
        (target.parent / members[0].name).rename(target)
    return target


def patch_once(path, before, after):
    text = path.read_text()
    if after in text:
        return
    if before not in text:
        raise RuntimeError('Unexpected upstream source: ' + str(path))
    path.write_text(text.replace(before, after))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error('Use 1..4 jobs to limit memory consumption')
    cmake = ROOT / 'tools/cmake/data/bin/cmake'
    if not cmake.exists():
        subprocess.run(['/usr/bin/python3.8', '-m', 'pip', 'install', '--target', str(ROOT/'tools'),
                        'cmake==3.27.9'], check=True)
    env = os.environ.copy()
    for key in ('CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
        env.pop(key, None)
    env.update(PATH='/usr/bin:/bin:/opt/ros/noetic/bin', ROS_VERSION='1', ROS_DISTRO='noetic',
               PYTHONNOUSERSITE='1', PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages',
               LD_LIBRARY_PATH=str(PREFIX/'lib')+':/opt/ros/noetic/lib')
    flags = {
        'fmt': ['-DFMT_TEST=OFF', '-DBUILD_SHARED_LIBS=ON'],
        'spdlog': ['-DSPDLOG_BUILD_SHARED=ON', '-DSPDLOG_FMT_EXTERNAL=ON'],
        'gtsam': ['-DGTSAM_BUILD_TESTS=OFF', '-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF',
                  '-DGTSAM_BUILD_UNSTABLE=ON', '-DGTSAM_WITH_TBB=OFF',
                  '-DGTSAM_USE_SYSTEM_EIGEN=ON', '-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF'],
        'gtsam_points': ['-DBUILD_WITH_CUDA=OFF', '-DBUILD_WITH_MARCH_NATIVE=OFF'],
        'glim': ['-DBUILD_WITH_CUDA=OFF', '-DBUILD_WITH_VIEWER=OFF', '-DBUILD_WITH_MARCH_NATIVE=OFF'],
        'glim_ros1': ['-DBUILD_WITH_CUDA=OFF', '-DBUILD_WITH_VIEWER=OFF'],
    }
    for name in ('fmt', 'spdlog', 'gtsam', 'gtsam_points', 'glim', 'glim_ros1'):
        source = fetch(name)
        if name == 'glim':
            graph_cpp = source/'src/glim/mapping/global_mapping_pose_graph.cpp'
            # Drain asynchronous loop validation before serializing the graph.
            # Also process the existing candidate buffer without waiting for new input.
            patch_once(source/'include/glim/mapping/global_mapping_pose_graph.hpp',
                       '  std::atomic_bool kill_switch;',
                       '  std::atomic_bool kill_switch;\n  std::atomic_size_t pending_loop_candidates{0};')
            patch_once(graph_cpp,
                       'void GlobalMappingPoseGraph::save(const std::string& path) {\n  optimize();',
                       'void GlobalMappingPoseGraph::save(const std::string& path) {\n'
                       '  while (pending_loop_candidates.load() != 0) {\n'
                       '    std::this_thread::sleep_for(std::chrono::milliseconds(10));\n'
                       '  }\n  optimize();')
            patch_once(graph_cpp, '  loop_candidates.insert(new_candidates);',
                       '  pending_loop_candidates += new_candidates.size();\n  loop_candidates.insert(new_candidates);')
            patch_once(graph_cpp, '    auto new_candidates = loop_candidates.get_all_and_clear_wait();',
                       '    auto new_candidates = candidates_buffer.empty() ? loop_candidates.get_all_and_clear_wait()\n'
                       '                                                     : loop_candidates.get_all_and_clear();')
            patch_once(graph_cpp, '      candidates_buffer.resize(params.loop_candidate_buffer_size);',
                       '      pending_loop_candidates -= candidates_buffer.size() - params.loop_candidate_buffer_size;\n'
                       '      candidates_buffer.resize(params.loop_candidate_buffer_size);')
            patch_once(graph_cpp, '    detected_loops.insert(factors);',
                       '    detected_loops.insert(factors);\n    pending_loop_candidates -= candidates.size();')
            # Expose accepted constraints in the run log for field verification.
            patch_once(graph_cpp,
                       '      // Create factor.\n',
                       '      logger->info("LOOP_ACCEPTED target={} source={} overlap={}",\n'
                       '                   candidates[i].target->submap->id, candidates[i].source->submap->id, inlier_fractions[i]);\n'
                       '      // Create factor.\n')
        if name == 'glim_ros1':
            # Upstream unconditionally installs a viewer target disabled in CPU/headless builds.
            patch_once(source/'CMakeLists.txt',
                       'install(TARGETS glim_ros rviz_viewer glim_rosnode glim_rosbag offline_viewer\n',
                       'install(TARGETS glim_ros rviz_viewer glim_rosnode glim_rosbag\n')
            # Keep each run's graph and trajectory instead of overwriting /tmp/dump.
            patch_once(source/'src/glim_rosnode.cpp', 'glim_ros->save("/tmp/dump");',
                       'glim_ros->save(nh.param<std::string>("dump_path", "/tmp/dump"));')
            patch_once(source/'src/glim_rosbag.cpp', 'glim_ros.save("/tmp/dump");',
                       'glim_ros.save(nh.param<std::string>("dump_path", "/tmp/dump"));')
            # A graph update can optimize existing submaps without adding one.
            # Upstream push_back duplicates the last cloud and indexes past poses.
            patch_once(source/'src/glim_ros/rviz_viewer.cpp',
                       'invoke([this, latest_submap, submap_poses] {\n    this->submaps.push_back(latest_submap->frame);',
                       'invoke([this, submaps, submap_poses] {\n'
                       '    this->submaps.clear();\n'
                       '    for (const auto& submap : submaps) this->submaps.push_back(submap->frame);')
        build = ROOT / 'build' / name
        command = [str(cmake), '-S', str(source), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release',
                   '-DCMAKE_CXX_FLAGS_RELEASE=-O2 -DNDEBUG', '-DCMAKE_INSTALL_PREFIX='+str(PREFIX),
                   '-DCMAKE_PREFIX_PATH='+str(PREFIX)+';/opt/ros/noetic',
                   '-DCMAKE_INSTALL_RPATH='+str(PREFIX/'lib')+';/opt/ros/noetic/lib',
                   '-DPYTHON_EXECUTABLE=/usr/bin/python3.8'] + flags[name]
        print('Building', name, flush=True)
        subprocess.run(command, env=env, check=True)
        subprocess.run([str(cmake), '--build', str(build), '-j', str(args.jobs)], env=env, check=True)
        subprocess.run([str(cmake), '--install', str(build)], env=env, check=True)
    (PREFIX/'rover_build.json').write_text(json.dumps(
        dict(sources=SOURCES, cuda=False, viewer='RViz', python='/usr/bin/python3.8'), indent=2)+'\n')
    print('GLIM installed:', PREFIX/'lib/glim_ros/glim_rosnode')


if __name__ == '__main__':
    main()
