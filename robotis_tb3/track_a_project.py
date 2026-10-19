"""Track A baseline: rendered camera + encoders -> EKF -> estimated-state feedback.
LiDAR rays -> inflated occupancy grid -> A* route. Ground truth only in evaluation.
"""
import argparse
import json
import hashlib
import shutil
import time
from pathlib import Path
from threading import Event
import cv2
import mujoco
import numpy as np
from camera_navigation import MarkerCamera
from turtlebot_model import (load_scene, WheelIO, twist_to_wheel_rates,
                            step_pose_from_encoder, wrap_angle)
from track_a_estimation import PoseEKF, marker_measurement
from track_a_navigation import OccupancyPlanner, StateFeedbackTracker
from track_a_sensors import build_room, Lidar, CalibratedMarkerCamera, evaluation_pose, evaluation_collision, WIDTH, HEIGHT


def run(args):
    rng=np.random.default_rng(args.seed)
    model=build_room(args.obstacle_shift,args.friction)
    _,spec=load_scene()
    data=mujoco.MjData(model)
    wheels=WheelIO(model,data,spec)
    for _ in range(500):
        mujoco.mj_step(model,data)
    # Known nominal start with uncertain prior; no simulator-pose initialisation.
    initial=np.array([.025,-.02,.025])
    ekf=PoseEKF(initial,spec,args.r_scale,args.q_scale)
    odometry=initial.copy()
    lidar=Lidar(model)
    planner=OccupancyPlanner()
    tracker=StateFeedbackTracker()
    detector=CalibratedMarkerCamera()
    previous=wheels.read_angles()
    records=[]
    output=Path(args.output).resolve()
    output.mkdir(parents=True,exist_ok=True)
    source_dir=output/'source-snapshot'
    source_dir.mkdir(exist_ok=True)
    sources=[Path(__file__), *(Path(__file__).parent/name for name in
             ('track_a_sensors.py','track_a_estimation.py','track_a_navigation.py',
              'turtlebot_model.py','camera_navigation.py','turtlebot3_waffle_pi.xml'))]
    manifest={}
    for source in sources:
        shutil.copy2(source,source_dir/source.name)
        manifest[source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
    (output/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    viewer=None
    quit_event=Event()
    if not args.headless:
        from mujoco import viewer as mjviewer
        viewer=mjviewer.launch_passive(model,data,key_callback=lambda k:quit_event.set() if k in (81,256) else None)
        viewer.cam.lookat[:]=[1.4,0,.15]
        viewer.cam.distance=4.5
        viewer.cam.azimuth=90
        viewer.cam.elevation=-60
    collided=False
    status='STARTING'
    render_options=mujoco.MjvOption()
    render_options.geomgroup[:]=1
    if viewer is not None:
        viewer.opt.geomgroup[:]=1
    try:
        with mujoco.Renderer(model,height=HEIGHT,width=WIDTH) as renderer:
            for frame in range(int(args.seconds*10)):
                tick=time.monotonic()
                if viewer is not None and (not viewer.is_running() or quit_event.is_set()):
                    break
                measured_angles=np.round(wheels.read_angles()*1024/(2*np.pi))*(2*np.pi/1024)
                increments=(measured_angles-previous)*np.array([1+args.encoder_bias,1-args.encoder_bias])
                previous=measured_angles
                ekf.predict(increments)
                odometry=step_pose_from_encoder(odometry,*increments,spec)
                renderer.update_scene(data,camera='navigation',scene_option=render_options)
                rgb=renderer.render()
                if args.pixel_noise:
                    rgb=np.clip(rgb.astype(float)+rng.normal(0,args.pixel_noise,rgb.shape),0,255).astype(np.uint8)
                observation,corners,ids=detector.observe(rgb)
                if rng.random()<args.dropout:
                    observation=None
                corrected=ekf.correct(observation)
                scan=lidar.read(model,data,rng,args.lidar_noise)
                planner.update(ekf.pose,lidar.angles,scan)
                path=planner.plan(ekf.pose[:2],tracker.goal)
                speed,turn,status=tracker.command(ekf.pose,path,lidar.angles,scan,corrected)
                rates=wheels.set_rates(*twist_to_wheel_rates(speed,turn,spec))
                # Evaluation only. These values are logged after control computation.
                truth=evaluation_pose(model,data)
                error=ekf.pose-truth
                error[2]=wrap_angle(error[2])
                record={'time_s':frame*.1,'estimate':ekf.pose.tolist(),'truth':truth.tolist(),
                        'odometry':odometry.tolist(),'covariance':ekf.P.tolist(),
                        'encoder_increments':increments.tolist(),
                        'camera_measurement':None if observation is None else observation.tolist(),'camera_accepted':bool(corrected),
                        'wheel_commands':rates.tolist(),'status':status,
                        'nis':ekf.last_nis if corrected else None,
                        'nees':float(error@np.linalg.solve(ekf.P,error)),
                        'path':np.asarray(path).tolist()}
                records.append(record)
                if frame%50==0 or tracker.arrived:
                    print(f'{frame*.1:5.1f}s {status} estimate={ekf.pose.round(3)} camera={corrected}',flush=True)
                if frame==0:
                    cv2.imwrite(str(output/'camera-start.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
                for _ in range(5):
                    wheels.step_update()
                    collided=collided or evaluation_collision(model,data)
                if viewer is not None:
                    viewer.sync()
                    inset=cv2.resize(rgb,(320,240))
                    viewport=viewer.viewport
                    viewer.set_images((mujoco.MjrRect(viewport.left+10,viewport.bottom+10,320,240),inset))
                    viewer.set_texts((None,None,f'{status}\nEKF x={ekf.pose[0]:.2f} y={ekf.pose[1]:.2f}\nQ: stop',None))
                    time.sleep(max(0,.1-(time.monotonic()-tick)))
                if collided or tracker.arrived:
                    break
            wheels.set_rates(0,0)
            for _ in range(50):
                wheels.step_update()
            renderer.update_scene(data,camera='navigation',scene_option=render_options)
            cv2.imwrite(str(output/'camera-finish.png'),cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR))
            truth=evaluation_pose(model,data)
            final_range=marker_measurement(truth)[0]
            errors=np.array([np.array(r['estimate'])-r['truth'] for r in records])
            odo_errors=np.array([np.array(r['odometry'])-r['truth'] for r in records])
            result={'arrived':bool(tracker.arrived),'collision':bool(collided),
                    'success':bool(tracker.arrived and not collided and abs(final_range-.55)<.08 and abs(truth[1])<.08 and abs(truth[2])<.15),
                    'simulation_seconds':len(records)*.1,'seed':args.seed,
                    'ekf_position_rmse_m':float(np.sqrt(np.mean(np.sum(errors[:,:2]**2,axis=1)))),
                    'odometry_position_rmse_m':float(np.sqrt(np.mean(np.sum(odo_errors[:,:2]**2,axis=1)))),
                    'final_camera_distance_truth_m':float(final_range),
                    'final_lateral_error_truth_m':float(abs(truth[1])),
                    'final_heading_error_truth_deg':float(abs(np.rad2deg(truth[2]))),
                    'camera_updates':ekf.accepted,'camera_rejections':ekf.rejected,
                    'mean_nees':float(np.mean([r['nees'] for r in records])),
                    'parameters':vars(args),'final_status':status}
            np.save(output/'occupancy.npy',planner.occupied)
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            (output/'trace.json').write_text(json.dumps(records,indent=2)+'\n')
            print(json.dumps(result),flush=True)
            if viewer is not None:
                viewer.set_texts((None,None,f'{status}\nRobot stopped. Q: close',None))
                while viewer.is_running() and not quit_event.is_set():
                    viewer.sync()
                    time.sleep(.05)
    finally:
        wheels.set_rates(0,0)
        if viewer is not None:
            viewer.close()
    return result


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--output',default='track-a-output')
    p.add_argument('--seconds',type=float,default=110)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--obstacle-shift',type=float,default=0)
    p.add_argument('--friction',type=float,default=1)
    p.add_argument('--encoder-bias',type=float,default=.002)
    p.add_argument('--dropout',type=float,default=0)
    p.add_argument('--pixel-noise',type=float,default=0)
    p.add_argument('--lidar-noise',type=float,default=.006)
    p.add_argument('--r-scale',type=float,default=1)
    p.add_argument('--q-scale',type=float,default=1)
    return p


if __name__=='__main__':
    args=parser().parse_args()
    result=run(args)
    if args.headless and not result['success']:
        raise SystemExit('Trial failed; see result.json and trace.json')
