#!/usr/bin/env python3
"""Host entry: existing simulation container + disposable NVIDIA/VirtualGL GUI."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

REPO=Path(__file__).resolve().parents[3]
IMAGE='s622-c43-gpu-viewer:virtualgl-3.1.5'


def docker(*args,**kwargs):
    return subprocess.run(['docker',*args],check=True,**kwargs)


def ensure_image(container):
    base=docker('inspect',container,'--format','{{.Image}}',capture_output=True,text=True).stdout.strip()
    # Include the base ID in the local tag so a changed simulation image rebuilds.
    image=IMAGE+'-'+base.removeprefix('sha256:')[:12]
    probe=subprocess.run(['docker','image','inspect',image],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    if probe.returncode:
        context=REPO/'src/gz_launch/docker/gpu_viewer'
        docker('build','--build-arg','BASE_IMAGE='+base,'-t',image,str(context))
    return image


def stop_lab(container,out):
    # Match the exact experiment directory, never a general Gazebo process name.
    code='''import os,signal,subprocess,sys
from pathlib import Path
target=sys.argv[1]
for row in subprocess.check_output(["ps","-eo","pid=,args="],text=True).splitlines():
    parts=row.strip().split(None,1)
    if len(parts)!=2:continue
    pid,args=parts; argv=args.split()
    if len(argv)>=4 and Path(argv[1]).name=="c43_force_lab.py" and argv[2]==target:
        try:os.kill(int(pid),signal.SIGINT)
        except ProcessLookupError:pass
'''
    subprocess.run(['docker','exec',container,'python3','-c',code,out],check=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container',default='s622_yep')
    parser.add_argument('--build-only',action='store_true')
    parser.add_argument('--fps',type=int,default=30)
    args=parser.parse_args()
    if not 1<=args.fps<=120:parser.error('--fps must be between 1 and 120')
    image=ensure_image(args.container)
    if args.build_only:
        print('GPU viewer image:',image,flush=True);return 0
    auth=Path(os.environ.get('XAUTHORITY',str(Path.home()/'.Xauthority'))).resolve()
    if not auth.is_file():raise RuntimeError('Current desktop XAUTHORITY is not readable: '+str(auth))
    run_id='gpu_gui_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%f')
    root=REPO/'log'/('c43_executor_gpu_'+datetime.now(timezone.utc).strftime('%Y%m%d'))
    root.mkdir(parents=True,exist_ok=True)
    out=root/run_id
    container_out='/workspace/'+str(out.relative_to(REPO))
    viewer_name='c43-vgl-'+run_id.lower()
    lab=None;viewer=None
    def terminate(_sig,_frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,terminate)
    with (root/(run_id+'_server_console.log')).open('w') as server_log, \
         (root/(run_id+'_viewer.log')).open('w') as viewer_log:
        try:
            lab=subprocess.Popen(['docker','exec','-w','/workspace',
                '-e','LIBGL_ALWAYS_SOFTWARE=1','-e','__GLX_VENDOR_LIBRARY_NAME=mesa',
                args.container,'bash','-lc',
                'source /opt/ros/humble/setup.bash; source install/setup.bash; exec ros2 run gz_launch c43_run_contact_transport.py "$@"',
                'c43_gpu_server','--backend','qp-executor','--external-gui','--keep-open',
                '--start-delay','5','--out',container_out],stdout=server_log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+60
            while True:
                try:
                    partition=json.loads((out/'conditions.json').read_text())['IGN_PARTITION']
                    break
                except (FileNotFoundError,json.JSONDecodeError,KeyError):
                    pass
                if lab.poll() is not None:raise RuntimeError('Server entry exited; inspect '+str(server_log.name))
                if time.monotonic()>deadline:raise RuntimeError('Experiment configuration did not appear')
                time.sleep(.2)
            command=['docker','run','--rm','--name',viewer_name,'--gpus','all',
                '--device','/dev/nvidia0','--device','/dev/nvidiactl','--device','/dev/nvidia-modeset',
                '--network','host','--ipc','host','-e','DISPLAY='+os.environ.get('DISPLAY',':4'),
                '-e','XAUTHORITY=/tmp/c43-xauth','-e','VGL_FPS='+str(args.fps),
                '-e','IGN_PARTITION='+partition,
                '-e','IGN_GAZEBO_RESOURCE_PATH=/workspace/install/share/gz_launch/models',
                '--mount','type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly',
                '--mount','type=bind,src='+str(auth)+',dst=/tmp/c43-xauth,readonly',
                '--mount','type=bind,src='+str(REPO)+',dst=/workspace,readonly',
                '--entrypoint','/bin/bash',image,'-lc',
                'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; '
                'exec /opt/VirtualGL/bin/vglrun -d egl -c proxy ign gazebo -g -v 4']
            (root/(run_id+'_viewer.json')).write_text(json.dumps(dict(
                image=image,viewer_container=viewer_name,partition=partition,
                experiment=str(out),display=os.environ.get('DISPLAY',':4'),fps=args.fps,
                renderer='NVIDIA EGL through VirtualGL proxy; confirm live renderer and visible image'),indent=2)+'\n')
            viewer=subprocess.Popen(command,stdout=viewer_log,stderr=subprocess.STDOUT)
            print('Experiment logs:',out,flush=True)
            print('GPU viewer:',viewer_name,flush=True)
            print('Closing the GPU window or Ctrl+C cleans up only this experiment.',flush=True)
            while viewer.poll() is None:
                if lab.poll() is not None:raise RuntimeError('Simulation server exited unexpectedly')
                time.sleep(.5)
            return viewer.returncode
        except KeyboardInterrupt:
            return 130
        finally:
            if viewer is not None:
                subprocess.run(['docker','stop','--time','3',viewer_name],
                    stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=False)
                try:viewer.wait(timeout=8)
                except subprocess.TimeoutExpired:pass
            if lab is not None:
                stop_lab(args.container,container_out)
                try:lab.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    print('Experiment cleanup is still in progress:',container_out,flush=True)


if __name__=='__main__':raise SystemExit(main())
