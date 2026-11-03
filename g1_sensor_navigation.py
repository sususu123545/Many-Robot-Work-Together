"""Sensor-only forest navigation: leg/IMU odometry, depth registration and local planning.

This module deliberately has no simulator imports, world-body state, known tree
coordinates, scene queries, or reference-route imports. Its pose origin is the
measured starting mark, and its destination is a mission-level goal.
"""
from __future__ import annotations
import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree
from g1_rgbd_sensor import CAMERA_POSITION, optical_to_base, unproject, fit_capture_ground


def rotation(roll=0., pitch=0., yaw=0.):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


class LegKinematics:
    """Forward kinematics from vendor URDF and measured joint angles only."""
    def __init__(self, urdf):
        self.chains = []
        nodes = ET.parse(urdf).getroot()
        parents = {j.find("child").get("link"): j for j in nodes.findall("joint")}
        for side in ("left", "right"):
            chain, link = [], f"{side}_ankle_roll_link"
            while link != "pelvis":
                j = parents[link]
                origin = j.find("origin")
                xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
                rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
                axis = j.find("axis")
                axis = np.fromstring(axis.get("xyz"), sep=" ") if axis is not None else np.array([1., 0, 0])
                chain.append((j.get("name"), xyz, rotation(*rpy), axis, j.get("type")))
                link = j.find("parent").get("link")
            self.chains.append(chain[::-1])

    def feet(self, joint_positions):
        result = []
        for chain in self.chains:
            r, t = np.eye(3), np.zeros(3)
            for name, xyz, origin_r, axis, kind in chain:
                t += r @ xyz
                r = r @ origin_r
                if kind != "fixed":
                    a = float(joint_positions.get(name, 0.))
                    ax = axis/np.linalg.norm(axis)
                    k = np.array([[0,-ax[2],ax[1]], [ax[2],0,-ax[0]], [-ax[1],ax[0],0]])
                    r = r @ (np.eye(3)+math.sin(a)*k+(1-math.cos(a))*(k@k))
            result.append(t+r@np.array([.025, 0., -.035]))
        return np.asarray(result)

    def contact_samples(self, joint_positions, joint_velocities, level_rotation):
        points, velocities = [], []
        corners=np.array([[-.045,-.035,-.035],[-.045,.035,-.035],
                          [.125,-.035,-.035],[.125,.035,-.035]])
        for chain in self.chains:
            r,t=np.eye(3),np.zeros(3); axes=[]; origins=[]; speeds=[]
            for name,xyz,origin_r,axis,kind in chain:
                t=t+r@xyz; r=r@origin_r
                if kind!="fixed":
                    ax=axis/np.linalg.norm(axis)
                    axes.append(r@ax); origins.append(t.copy()); speeds.append(joint_velocities.get(name,0.))
                    a=float(joint_positions.get(name,0.))
                    k=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
                    r=r@(np.eye(3)+math.sin(a)*k+(1-math.cos(a))*(k@k))
            sole=corners@r.T+t
            point=sole[np.argmin((sole@level_rotation.T)[:,2])]
            velocity=np.sum(np.cross(np.array(axes),point-np.array(origins))*np.array(speeds)[:,None],axis=0)
            points.append(point); velocities.append(velocity)
        return np.array(points),np.array(velocities)


class SensorOdometry:
    def __init__(self, urdf, start=(-1.2, 0.0, 0.0)):
        self.kinematics = LegKinematics(urdf)
        self.pose = np.array(start, dtype=float)
        self.velocity = np.zeros(3)
        self.roll = self.pitch = 0.
        self.height = .79
        self.previous_feet = None
        self.previous_cloud = self.previous_pose = None
        self.visual_updates = self.visual_rejected = 0
        self.last_icp_rmse = None
        self.distance = 0.
        self.previous_rgbd = None
        self.pnp_updates = self.pnp_rejected = 0
        self.last_feature_seconds = 0.
        self.feature_accepted = False
        self.lane_lines = {}
        self.lane_updates = 0

    def correct_corridor(self, points, seconds):
        """Learn parallel fence landmarks from RGB-D; constrain lateral drift.

        Fence positions are measured during initial standing, never supplied
        from the USD or course builder. Mission X follows the starting heading.
        """
        if len(points)<40: return
        aligned=points@rotation(yaw=self.pose[2])[:2,:2].T
        candidates={}
        for side in (-1,1):
            sample=aligned[(side*aligned[:,1]>.55)&(side*aligned[:,1]<3.5)]
            if len(sample)<30:continue
            bins,counts=np.unique(np.rint(sample[:,1]/.04).astype(int),return_counts=True)
            best=None
            for index in np.argsort(counts)[-12:]:
                peak=bins[index]*.04
                row=sample[np.abs(sample[:,1]-peak)<.065]
                coverage=len(np.unique(np.rint(row[:,0]/.25).astype(int)))
                if coverage<8:continue
                value=float(np.median(row[:,1]))
                if best is None or coverage>best[0]:best=(coverage,value)
            if best is not None:candidates[side]=best[1]
        corrections=[]
        for side,relative in candidates.items():
            if side not in self.lane_lines:
                if seconds<2.:self.lane_lines[side]=relative+self.pose[1]
            elif seconds<2.:
                self.lane_lines[side]+=.08*(relative+self.pose[1]-self.lane_lines[side])
            else:
                innovation=self.lane_lines[side]-relative-self.pose[1]
                if abs(innovation)<.5:corrections.append(innovation)
        if corrections:
            self.pose[1]+=np.clip(.3*np.median(corrections),-.025,.025)
            self.lane_updates+=1

    def visual_register(self, frame):
        """RGB feature tracks with metric depth; no scene labels or world poses."""
        import cv2
        gray=cv2.cvtColor(frame["rgb"],cv2.COLOR_RGB2GRAY)
        keypoints,descriptors=cv2.ORB_create(nfeatures=900,fastThreshold=7,edgeThreshold=8).detectAndCompute(gray,None)
        previous=self.previous_rgbd
        accepted=False
        if previous is not None and descriptors is not None and previous[1] is not None:
            kp0,desc0,depth0,k0,pose0,rpy0=previous
            pairs=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(desc0,descriptors,k=2)
            matches=[p[0] for p in pairs if len(p)==2 and p[0].distance<.75*p[1].distance]
            objects=[]; pixels=[]; current_depth=[]
            for match in matches:
                u,v=kp0[match.queryIdx].pt; uc,vc=keypoints[match.trainIdx].pt
                d=float(depth0[int(round(v)),int(round(u))])
                dc=float(frame["depth"][int(round(vc)),int(round(uc))])
                if not (.3<d<7.5 and .3<dc<7.5): continue
                objects.append([(u-k0[0,2])*d/k0[0,0],(v-k0[1,2])*d/k0[1,1],d])
                pixels.append([uc,vc]); current_depth.append(dc)
            if len(objects)>=10:
                objects=np.array(objects,dtype=np.float32); pixels=np.array(pixels,dtype=np.float32)
                ok,rvec,tvec,inliers=cv2.solvePnPRansac(objects,pixels,frame["K"],None,
                    iterationsCount=100,reprojectionError=1.8,confidence=.995,flags=cv2.SOLVEPNP_ITERATIVE)
                if ok and inliers is not None and len(inliers)>=9:
                    rc,_=cv2.Rodrigues(rvec); tc=tvec.ravel()
                    projected=objects@rc.T+tc
                    valid=inliers.ravel()
                    depth_error=np.median(np.abs(projected[valid,2]-np.array(current_depth)[valid]))
                    rb=optical_to_base(); tb=CAMERA_POSITION
                    motion_r=rb@rc.T@rb.T
                    motion_t=tb-rb@rc.T@tc-motion_r@tb
                    rw=rotation(rpy0[0],rpy0[1],pose0[2])
                    candidate=pose0[:2]+(rw@motion_t)[:2]
                    orientation=rw@motion_r
                    candidate_yaw=math.atan2(orientation[1,0],orientation[0,0])
                    if depth_error<.08 and np.linalg.norm(candidate-self.pose[:2])<.16 and abs(wrap(candidate_yaw-self.pose[2]))<.12:
                        # The metric optical-axis displacement is well constrained;
                        # leg odometry and IMU attitude constrain lateral/heading motion.
                        forward=rotation(yaw=self.pose[2])[:2,0]
                        self.pose[:2]+=.85*np.dot(candidate-self.pose[:2],forward)*forward
                        self.pnp_updates+=1; accepted=True
                        self.last_feature_seconds=frame["seconds"]
        if not accepted: self.pnp_rejected+=1
        self.feature_accepted=accepted
        self.previous_rgbd=(keypoints,descriptors,frame["depth"],frame["K"],self.pose.copy(),(self.roll,self.pitch))

    def predict(self, joints, gyro, gravity, dt, joint_velocities=None, attitude_yaw=None):
        gx, gy, gz = gravity
        self.roll = math.atan2(-gy, -gz)
        self.pitch = math.atan2(gx, math.hypot(gy, gz))
        level_rotation=rotation(self.roll,self.pitch)
        if joint_velocities is not None:
            feet,measured_foot_velocity=self.kinematics.contact_samples(joints,joint_velocities,level_rotation)
        else:
            feet=self.kinematics.feet(joints); measured_foot_velocity=None
        level = feet @ rotation(self.roll, self.pitch).T
        self.height = float(-np.min(level[:, 2]))
        if self.previous_feet is not None:
            foot_velocity = measured_foot_velocity if measured_foot_velocity is not None else (feet-self.previous_feet)/dt
            candidates = -(foot_velocity+np.cross(np.asarray(gyro), feet))
            heights = level[:, 2]-level[:, 2].min()
            # At this checkpoint the swing sole can be just millimetres above
            # the stance sole. Blending both would cancel forward leg velocity.
            weights = np.zeros(2)
            weights[int(np.argmin(heights))] = 1.
            v_body = np.sum(candidates*weights[:, None], axis=0)/max(1e-8, weights.sum())
            v_world = rotation(self.roll, self.pitch, self.pose[2]) @ v_body
            v_world = np.clip(v_world, -1.5, 1.5)
            self.pose[:2] += v_world[:2]*dt
            self.distance += np.linalg.norm(v_world[:2])*dt
            self.velocity[:2] += .15*(v_body[:2]-self.velocity[:2])
        yaw_rate = (gyro[1]*math.sin(self.roll)+gyro[2]*math.cos(self.roll))/max(.5,math.cos(self.pitch))
        self.pose[2] = wrap(self.pose[2]+yaw_rate*dt)
        if attitude_yaw is not None:
            self.pose[2]=wrap(self.pose[2]+.08*wrap(attitude_yaw-self.pose[2]))
        self.velocity[2] += .15*(yaw_rate-self.velocity[2])
        self.previous_feet = feet

    def register(self, cloud):
        """2D depth ICP of above-ground structure, seeded by leg/IMU motion."""
        if len(cloud) < 50:
            self.visual_rejected += 1
            return
        _,indices = np.unique(np.round(cloud/.035).astype(int), axis=0,return_index=True)
        current = cloud[indices]
        if self.previous_cloud is not None:
            prev_pose = self.previous_pose
            r0 = rotation(yaw=-prev_pose[2])[:2, :2]
            t = r0 @ (self.pose[:2]-prev_pose[:2])
            angle = wrap(self.pose[2]-prev_pose[2])
            seed_t, seed_angle = t.copy(), angle
            tree = cKDTree(self.previous_cloud)
            rmse, pairs = float("inf"), 0
            for _ in range(7):
                r = rotation(yaw=angle)[:2,:2]
                transformed = current @ r.T+t
                distances, indices = tree.query(transformed)
                valid = distances < .15
                pairs = int(valid.sum())
                if pairs < 35:
                    break
                a, b = transformed[valid], self.previous_cloud[indices[valid]]
                # Robust trim rejects moving legs and partially occluded edges.
                trim = distances[valid] <= np.quantile(distances[valid], .8)
                a, b = a[trim], b[trim]
                am, bm = a.mean(axis=0), b.mean(axis=0)
                u, _, vt = np.linalg.svd((a-am).T@(b-bm))
                dr = vt.T@u.T
                if np.linalg.det(dr) < 0:
                    vt[-1] *= -1
                    dr = vt.T@u.T
                dt = bm-dr@am
                t = dr@t+dt
                angle = wrap(angle+math.atan2(dr[1,0],dr[0,0]))
                rmse = float(np.sqrt(np.mean(np.sum((a@dr.T+dt-b)**2,axis=1))))
            self.last_icp_rmse = rmse if math.isfinite(rmse) else None
            if pairs >= 35 and rmse < .075 and np.linalg.norm(t-seed_t)<.10 and abs(wrap(angle-seed_angle))<.06:
                candidate = prev_pose[:2]+rotation(yaw=prev_pose[2])[:2,:2]@t
                correction=rotation(yaw=-self.pose[2])[:2,:2]@(candidate-self.pose[:2])
                # Long corridor walls poorly constrain forward translation.
                # Preserve encoder odometry along the corridor, correct lateral drift.
                correction*=np.array([.02,.20])
                if not getattr(self,"feature_accepted",False):
                    self.pose[:2] += rotation(yaw=self.pose[2])[:2,:2]@np.clip(correction,-.004,.004)
                self.visual_updates += 1
            else:
                self.visual_rejected += 1
        self.previous_cloud = current
        self.previous_pose = self.pose.copy()


class DepthObstacleMap:
    """Metric rolling obstacle grid built only from RGB/depth and estimated pose."""
    def __init__(self):
        self.cells = {}
        self.voxels = {}
        self.trees = []
        self.last_frame_seconds = -1.
        self.frame_count = 0
        self.points = np.empty((0,2))
        self.last_valid_fraction = 0.
        self.last_obstacle_points = 0
        self.elevation = np.empty((0,3))
        self.ground_fit_frames=0

    def update(self, frame, odom):
        depth, rgb = frame["depth"], frame["rgb"]
        self.last_valid_fraction = float(np.mean(np.isfinite(depth)&(depth>.2)&(depth<8)))
        if self.last_valid_fraction < .15:
            return False
        odom.visual_register(frame)
        optical, pixels = unproject(depth, frame["K"], stride=2)
        points_b = optical @ optical_to_base().T+CAMERA_POSITION
        capture_roll,capture_pitch,capture_height,ground_rmse=fit_capture_ground(
            points_b,pixels,depth.shape[0],odom.roll,odom.pitch,odom.height)
        if ground_rmse is not None:self.ground_fit_frames+=1
        frame["ground_frame"]=np.array([capture_roll,capture_pitch,capture_height,ground_rmse if ground_rmse is not None else np.nan])
        level = points_b @ rotation(capture_roll, capture_pitch).T
        level[:,2] += capture_height
        colors = rgb[pixels].astype(float)/255
        import cv2
        hsv_image=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
        # Anti-aliased leaf silhouettes are near-white but retain green hue.
        # A high saturation cutoff would turn those depth returns into walls.
        green_image=(hsv_image[...,0]>=28)&(hsv_image[...,0]<=100)&(hsv_image[...,1]>4)&(rgb[...,1].astype(float)>=.99*rgb[...,0])
        green_image=cv2.dilate(green_image.astype(np.uint8),np.ones((3,3),dtype=np.uint8),iterations=2).astype(bool)
        green=green_image[pixels]
        # Ignore leaves overhead, preserve trunks, stumps, roots, fence and new objects.
        # The pelvis camera can see swinging forearms in its near field.
        # Retain previously mapped close obstacles, but do not insert self returns.
        # The support plane is fitted from 0.7-4.5 m returns only. Beyond that
        # range terrain undulation is extrapolated, so both signs of height need
        # a distance-dependent threshold; otherwise ordinary bumps several
        # metres out become a wall of phantom obstacles and the lattice planner
        # reports no_clear_path before the robot reaches the terrain at all.
        # Measured on the moderate terrain, even pure ground returns reach
        # z = 0.144 at the 99th percentile, which is above the 0.10 obstacle
        # floor. Trees, posts and fence rails stay far above any threshold here.
        distance = level[:, 0]
        obstacle_height = np.minimum(.10 + .045*np.maximum(0., distance - 3.0), .35)
        is_obstacle = (level[:, 2] > obstacle_height) | ((level[:, 2] > .06) & (distance < 3.0))
        is_pit = (level[:, 2] < -.08) & (distance < 2.0)
        mask = (is_obstacle | is_pit) & (level[:, 2] < 1.35) & (distance > .65) & ((~green) | (level[:, 2] < .45))
        xy = level[mask,:2]
        odom.register(xy)
        fence=(level[:,2]>.25)&(level[:,2]<.90)&(level[:,0]>.7)&(level[:,0]<7.)&(~green)
        odom.correct_corridor(level[fence,:2],frame["seconds"])
        world = xy @ rotation(yaw=odom.pose[2])[:2,:2].T+odom.pose[:2]
        world_xyz=np.column_stack((world,level[mask,2]))
        vegetation=(level[:,2]>.45)&(level[:,2]<2.8)&(level[:,0]>.5)&green
        veg_xy=level[vegetation,:2]@rotation(yaw=odom.pose[2])[:2,:2].T+odom.pose[:2]
        veg_xyz=np.column_stack((veg_xy,level[vegetation,2]))
        vegetation_tree=cKDTree(veg_xyz) if len(veg_xyz) else None
        if vegetation_tree is not None and len(world_xyz):
            keep=(world_xyz[:,2]<.45)|(vegetation_tree.query(world_xyz)[0]>.22)
            world_xyz=world_xyz[keep]
        surface=(level[:,2]>-.20)&(level[:,2]<.20)&(level[:,0]>.65)&(level[:,0]<4.)&(~green)
        terrain_xy=level[surface,:2]@rotation(yaw=odom.pose[2])[:2,:2].T+odom.pose[:2]
        self.elevation=np.column_stack((terrain_xy,level[surface,2]))
        tall = (level[:,2]>.7)&(level[:,2]<2.8)&green&(level[:,0]>.5)
        tall_world = level[tall,:2] @ rotation(yaw=odom.pose[2])[:2,:2].T+odom.pose[:2]
        tall_world = tall_world[np.abs(tall_world[:,1])<1.65]
        # Connected components of geometric tall returns, not USD semantic labels.
        if len(tall_world):
            bins = np.unique(np.round(tall_world/.12).astype(int),axis=0)
            remaining = {tuple(v) for v in bins}
            while remaining:
                stack=[remaining.pop()]; group=[]
                while stack:
                    p=stack.pop(); group.append(p)
                    for dx in (-1,0,1):
                        for dy in (-1,0,1):
                            q=(p[0]+dx,p[1]+dy)
                            if q in remaining:
                                remaining.remove(q); stack.append(q)
                if len(group)<5: continue
                center=np.mean(group,axis=0)*.12
                bearing=center-odom.pose[:2]
                center+=.25*bearing/max(.1,np.linalg.norm(bearing))
                if abs(center[1])<.45: continue
                match=next((t for t in self.trees if np.linalg.norm(t-center)<.9),None)
                if match is None: self.trees.append(center)
                else: match += .08*(center-match)
        now=frame["seconds"]
        # Visibility-based clearing: a stale false return must not form a wall
        # for the whole memory window. Keep 3D cells to respect ray height.
        if self.voxels:
            keys=list(self.voxels)
            xyz=np.array(keys,dtype=float)*.08
            semantic_clear=np.zeros(len(keys),dtype=bool)
            if vegetation_tree is not None:
                semantic_clear=(xyz[:,2]>.45)&(vegetation_tree.query(xyz)[0]<.25)
            xyz[:,:2]-=odom.pose[:2]; xyz[:,2]-=capture_height
            body=xyz@rotation(capture_roll,capture_pitch,odom.pose[2])
            optical_existing=(body-CAMERA_POSITION)@optical_to_base()
            z=optical_existing[:,2]
            safe_z=np.maximum(.01,z)
            u=np.rint(frame["K"][0,0]*optical_existing[:,0]/safe_z+frame["K"][0,2]).astype(int)
            v=np.rint(frame["K"][1,1]*optical_existing[:,1]/safe_z+frame["K"][1,2]).astype(int)
            inside=(z>.2)&(u>=0)&(u<depth.shape[1])&(v>=0)&(v<depth.shape[0])
            for index in np.flatnonzero(inside):
                d=depth[v[index],u[index]]
                if np.isfinite(d) and (d>z[index]+.12 or (green_image[v[index],u[index]] and d>z[index]-.10)):
                    score,stamp=self.voxels[keys[index]]
                    self.voxels[keys[index]]=(score-3,stamp)
            for index in np.flatnonzero(semantic_clear):
                self.voxels[keys[index]]=(0,now)
        self.voxels={p:(score,stamp) for p,(score,stamp) in self.voxels.items()
                     if score>0 and now-stamp<8. and p[0]*.08>odom.pose[0]-.8}
        bins, counts=np.unique(np.round(world_xyz/.08).astype(int),axis=0,return_counts=True)
        for p,count in zip(bins,counts):
            if count>=2 and abs(p[1]*.08)<2.45:
                old=self.voxels.get(tuple(p),(0,now))[0]
                self.voxels[tuple(p)]=(min(8,old+2),now)
        confirmed=[p[:2] for p,(score,stamp) in self.voxels.items() if score>=3]
        self.points=np.unique(np.array(confirmed,dtype=float).reshape(-1,2),axis=0)*.08
        self.last_frame_seconds=now
        self.frame_count+=1
        self.last_obstacle_points=len(world)
        if odom.previous_rgbd is not None:
            previous=list(odom.previous_rgbd); previous[4]=odom.pose.copy(); odom.previous_rgbd=tuple(previous)
        return True

    def reference(self, x):
        # Slalom side comes from the observed tree's side of the corridor.
        value=np.zeros_like(np.asarray(x),dtype=float)
        for tree in self.trees:
            value += -math.copysign(.32,tree[1])*np.exp(-((np.asarray(x)-tree[0])/1.10)**2)
        return np.clip(value,-.6,.6)


class OnlineNavigator:
    """Receding-horizon lattice planner, with sensor timeout and clearance stop."""
    def __init__(self, speed=.22, lane_width=2.0):
        self.speed=speed
        self.command=np.zeros(3)
        self.phase="WAIT_FOR_RGBD"
        self.path=np.empty((0,2))
        self.last_plan=-1.
        self.settle_since=None
        self.min_clearance=float("inf")
        self.replans=self.blocked_plans=self.stale_stops=0
        self.last_reason=None
        self.lane_width=lane_width
        self.clear_path_since=None
        self.motion_anchor=None
        self.motion_anchor_time=0.
        self.boost_until=0.
        self.gait_restarts=0
        self.consecutive_stalls=0

    def plan(self, pose, obstacle_map):
        x,y,_=pose
        # Terminate the forest plan at its exit, not inside the next task's rocks.
        xs=np.arange(x, min(15.28,x+5.0)+.01,.35)
        if not len(xs):xs=np.array([x])
        half=self.lane_width/2-.35
        ys=np.arange(-half,half+.001,.05)
        grid=np.stack(np.meshgrid(xs,ys,indexing="ij"),axis=-1)
        ref=obstacle_map.reference(xs)
        if len(obstacle_map.points):
            clearance=cKDTree(obstacle_map.points).query(grid.reshape(-1,2))[0].reshape(grid.shape[:2])
        else: clearance=np.full(grid.shape[:2],4.)
        cost=1.3*(ys[None,:]-ref[:,None])**2+.06/np.maximum(.05,clearance-.30)**2
        cost[clearance<.50]=np.inf
        d=np.full_like(cost,np.inf); back=np.zeros_like(cost,dtype=int)
        d[0]=cost[0]+100*(ys-y)**2
        delta=ys[:,None]-ys[None,:]
        transition=4.0*delta**2
        transition[np.abs(delta)>.251]=np.inf
        for i in range(1,len(xs)):
            options=d[i-1][:,None]+transition
            back[i]=np.argmin(options,axis=0)
            d[i]=cost[i]+np.min(options,axis=0)
        if not np.isfinite(d[-1]).any():
            self.blocked_plans+=1; self.path=np.empty((0,2)); return
        idx=int(np.argmin(d[-1])); selected=[idx]
        for i in range(len(xs)-1,0,-1):
            idx=int(back[i,idx]); selected.append(idx)
        route_y=ys[np.array(selected[::-1])]
        self.path=np.column_stack((xs,route_y))
        self.replans+=1

    def update(self, odom, obstacle_map, seconds, dt=.02):
        desired=np.zeros(3)
        pose=odom.pose
        if seconds-obstacle_map.last_frame_seconds>.35:
            self.phase="WAIT_FOR_RGBD" if seconds<2. else "SENSOR_STOP"; self.last_reason="rgbd_stale"
            self.command[:]=0.
            if seconds>=2.: self.stale_stops+=1
            return tuple(self.command)
        if seconds-getattr(odom,"last_feature_seconds",seconds)>1.:
            self.phase="LOCALIZATION_STOP"; self.last_reason="visual_odometry_unavailable"
            self.command[:]=0.; return tuple(self.command)
        if seconds<2.0 or obstacle_map.frame_count<4:
            self.phase="STAND"
        elif self.phase in ("SETTLE","COMPLETE"):
            if seconds-self.settle_since>=3.: self.phase="COMPLETE"
        elif pose[0]>=15.28 and abs(pose[1])<self.lane_width/2-.32:
            self.phase="SETTLE"; self.settle_since=seconds
        else:
            if seconds-self.last_plan>=.2:
                self.plan(pose,obstacle_map); self.last_plan=seconds
            if len(self.path)==0:
                self.phase="BLOCKED_STOP"; self.last_reason="no_clear_path"
                self.clear_path_since=None
            else:
                if self.clear_path_since is None:self.clear_path_since=seconds
                if seconds-self.clear_path_since<.4:
                    self.phase="BLOCKED_STOP"; self.last_reason="confirming_clear_path"
                    self.command[:]=0.; return tuple(self.command)
                self.phase="CROSS"
                look=1.15
                target=np.array([pose[0]+look,np.interp(pose[0]+look,self.path[:,0],self.path[:,1])])
                angle=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
                rate=float(np.clip(max(.2,odom.velocity[0])*2*math.sin(angle)/look,-.14,.16))
                unbias=rate-.02
                ff=unbias/(.5 if unbias<0 else 1.)
                desired[0]=self.speed
                # Ease into the finish so the locomotion policy is already
                # moving slowly before the terminal settle phase begins.
                remaining=max(0.,15.5-float(pose[0]))
                desired[0]=min(self.speed,max(.12,self.speed*min(1.,remaining/1.5)))
                # The policy trains ang_vel_z within [-0.2, 0.2], but clamping
                # the request to that band measurably weakens slalom clearance
                # on flat ground (v12 touched an obstacle where v10 did not),
                # while the terrain fall in v4 happened *after* a BLOCKED_STOP
                # with zero command, i.e. it was standing balance, not steering.
                # Keep the wider band and treat terrain stability separately.
                desired[2]=np.clip(ff+.3*(rate-odom.velocity[2]),-.28,.25)
                if len(obstacle_map.points):
                    clearance=float(cKDTree(obstacle_map.points).query(pose[:2])[0])
                    self.min_clearance=min(self.min_clearance,clearance)
                    if clearance<.45:
                        desired[:]=0; self.phase="CLEARANCE_STOP"; self.last_reason="obstacle_too_close"
                        self.command[:]=0.; return tuple(self.command)
                if self.motion_anchor is None:
                    self.motion_anchor=float(pose[0]); self.motion_anchor_time=seconds
                elif seconds-self.motion_anchor_time>=1.2:
                    progress=pose[0]-self.motion_anchor
                    if progress<.045 and seconds>4. and remaining>1.5:
                        self.boost_until=seconds+.8
                        self.gait_restarts+=1
                        self.consecutive_stalls+=1
                    else:self.consecutive_stalls=0
                    self.motion_anchor=float(pose[0]); self.motion_anchor_time=seconds
                if self.consecutive_stalls>=5:
                    self.phase="GAIT_STOP"; self.last_reason="gait_no_progress"
                    self.command[:]=0.; return tuple(self.command)
                if seconds<self.boost_until and remaining>1.5:
                    desired[0]=.28; desired[2]=np.clip(desired[2],-.20,.20)
                    self.phase="GAIT_RESTART"
        limit=np.array([.3,.2,.6])*dt
        self.command+=np.clip(desired-self.command,-limit,limit)
        return tuple(self.command)
