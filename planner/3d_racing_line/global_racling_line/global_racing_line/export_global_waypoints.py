"""
3D racing line CSV → UNICORN 호환 global waypoints JSON 변환.

입력: data/global_racing_lines/{raceline_name} (time-optimal 또는 sector-tuned CSV)
출력: data/global_racing_lines/{output_name} (UNICORN WpntArray 포맷 JSON)

변환 과정:
  1. curvilinear (s_opt, n_opt) → Cartesian (x, y, z) via sn2cartesian
  2. 레이싱라인 실제 arc length 계산
  3. 실제 arc length 기준 등간격(0.1m) 재보간
  4. heading, kappa는 재보간된 x,y 좌표에서 직접 계산
  5. s_m = 레이싱라인 실제 arc length, d_m = 0
  6. RViz markers 생성 (centerline, raceline, trackbounds)

Usage:
  python global_racing_line/export_global_waypoints.py
"""
import os
import sys
import json
import numpy as np
import pandas as pd

params = {
    'track_name': 'experiment_3d_2_3d_smoothed_waypoint1.csv',
    'raceline_name': 'experiment_3d_2_3d_smoothed_waypoint1_timeoptimal.csv',
    'output_name': 'experiment_3d_2_3d_smoothed_waypoint1_timeoptimal.json',
    'waypoint_spacing': 0.1,  # 재보간 간격 [m]
}

# paths
dir_path = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(dir_path, '..', 'data')
track_path = os.path.join(data_path, 'smoothed_track_data', params['track_name'])
raceline_path = os.path.join(data_path, 'global_racing_lines', params['raceline_name'])
output_path = os.path.join(data_path, 'global_racing_lines', params['output_name'])
sys.path.append(os.path.join(dir_path, '..', 'src'))

from track3D import Track3D


# ── RViz Marker 생성 헬퍼 ──

def _make_marker_template():
    return {
        'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': 'map'},
        'ns': '', 'id': 0, 'type': 2, 'action': 0,
        'pose': {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1},
        },
        'scale': {'x': 0.05, 'y': 0.05, 'z': 0.05},
        'color': {'r': 0.0, 'g': 0.0, 'b': 0.0, 'a': 1.0},
        'lifetime': {'secs': 0, 'nsecs': 0},
        'frame_locked': False,
        'points': [], 'colors': [], 'text': '',
        'mesh_resource': '', 'mesh_use_embedded_materials': False,
    }


def _build_sphere_markers(xs, ys, zs, r, g, b, scale=0.05):
    """type=2 (SPHERE) markers."""
    markers = []
    for i in range(len(xs)):
        m = _make_marker_template()
        m['id'] = i
        m['type'] = 2  # SPHERE
        m['pose']['position'] = {'x': float(xs[i]), 'y': float(ys[i]), 'z': float(zs[i])}
        m['scale'] = {'x': scale, 'y': scale, 'z': scale}
        m['color'] = {'r': r, 'g': g, 'b': b, 'a': 1.0}
        markers.append(m)
    return markers


def _build_cylinder_markers(xs, ys, zs, vs, r, g, b):
    """type=3 (CYLINDER) markers. 높이 = 속도 비례 (scale_factor=0.1317)."""
    scale_factor = 0.1317
    markers = []
    for i in range(len(xs)):
        m = _make_marker_template()
        m['id'] = i
        m['type'] = 3  # CYLINDER
        height = float(vs[i]) * scale_factor
        m['pose']['position'] = {'x': float(xs[i]), 'y': float(ys[i]), 'z': height / 2.0}
        m['scale'] = {'x': 0.1, 'y': 0.1, 'z': height}
        m['color'] = {'r': r, 'g': g, 'b': b, 'a': 1.0}
        markers.append(m)
    return markers


def _build_trackbounds_markers(track):
    """좌/우 트랙 경계 sphere markers. 좌=보라(0.5,0,0.5), 우=노랑(0.5,1,0)."""
    markers = []
    marker_id = 0

    for k in range(len(track.s)):
        s = track.s[k]
        theta = track.theta[k]
        x_c, y_c, z_c = track.x[k], track.y[k], track.z[k]

        # 왼쪽 경계
        w_l = track.w_tr_left[k]
        x_l = x_c - w_l * np.sin(theta)
        y_l = y_c + w_l * np.cos(theta)

        m = _make_marker_template()
        m['id'] = marker_id
        m['pose']['position'] = {'x': float(x_l), 'y': float(y_l), 'z': float(z_c)}
        m['color'] = {'r': 0.5, 'g': 0.0, 'b': 0.5, 'a': 1.0}
        markers.append(m)
        marker_id += 1

        # 오른쪽 경계
        w_r = track.w_tr_right[k]  # 음수
        x_r = x_c - w_r * np.sin(theta)
        y_r = y_c + w_r * np.cos(theta)

        m = _make_marker_template()
        m['id'] = marker_id
        m['pose']['position'] = {'x': float(x_r), 'y': float(y_r), 'z': float(z_c)}
        m['color'] = {'r': 0.5, 'g': 1.0, 'b': 0.0, 'a': 1.0}
        markers.append(m)
        marker_id += 1

    return markers


# ── 메인 export ──

def export_waypoints():
    # 트랙 로드
    track = Track3D(path=track_path)

    # racing line 로드
    rl = pd.read_csv(raceline_path)
    s_opt = rl['s_opt'].values
    v_opt = rl['v_opt'].values
    n_opt = rl['n_opt'].values
    chi_opt = rl['chi_opt'].values
    ax_opt = rl['ax_opt'].values
    laptime = rl['laptime'].iloc[0]

    # ── Step 1: curvilinear → Cartesian 변환 ──
    n_points = len(s_opt)
    x_raw = np.zeros(n_points)
    y_raw = np.zeros(n_points)
    z_raw = np.zeros(n_points)

    for k in range(n_points):
        cart = track.sn2cartesian(s_opt[k], n_opt[k])
        x_raw[k] = float(cart[0])
        y_raw[k] = float(cart[1])
        z_raw[k] = float(cart[2])

    # ── Step 2: 레이싱라인 실제 arc length 계산 ──
    ds_real = np.sqrt(np.diff(x_raw)**2 + np.diff(y_raw)**2 + np.diff(z_raw)**2)
    arc_raw = np.zeros(n_points)
    arc_raw[1:] = np.cumsum(ds_real)
    total_length = arc_raw[-1]

    # ── Step 3: 실제 arc length 기준 등간격 재보간 ──
    spacing = params['waypoint_spacing']
    n_new = int(total_length / spacing) + 1
    arc_new = np.linspace(0, total_length, n_new)

    x_new = np.interp(arc_new, arc_raw, x_raw)
    y_new = np.interp(arc_new, arc_raw, y_raw)
    z_new = np.interp(arc_new, arc_raw, z_raw)
    v_new = np.interp(arc_new, arc_raw, v_opt)
    ax_new = np.interp(arc_new, arc_raw, ax_opt)
    # s_opt도 보간 (트랙 경계 조회용)
    s_opt_new = np.interp(arc_new, arc_raw, s_opt)
    n_opt_new = np.interp(arc_new, arc_raw, n_opt)

    # ── Step 4: heading, kappa 계산 (재보간된 x,y에서 직접) ──
    dx = np.diff(x_new)
    dy = np.diff(y_new)
    psi = np.arctan2(dy, dx)
    psi = np.append(psi, psi[-1])

    # 곡률: dpsi/ds
    dpsi = np.diff(psi)
    dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi  # wrap to [-pi, pi]
    ds_new = np.diff(arc_new)
    kappa = dpsi / ds_new
    kappa = np.append(kappa, kappa[-1])

    # ── Step 5: 트랙 경계 거리 계산 + waypoints 생성 ──
    waypoints = []
    for k in range(n_new):
        s = s_opt_new[k]
        n = n_opt_new[k]

        w_tr_left = float(track.w_tr_left_interpolator(s))
        w_tr_right = float(track.w_tr_right_interpolator(s))
        d_left = w_tr_left - n
        d_right = -w_tr_right + n

        wpnt = {
            'id': k,
            's_m': float(arc_new[k]),
            'd_m': 0.0,
            'x_m': float(x_new[k]),
            'y_m': float(y_new[k]),
            'z_m': float(z_new[k]),
            'd_right': float(abs(d_right)),
            'd_left': float(abs(d_left)),
            'psi_rad': float(psi[k]),
            'kappa_radpm': float(kappa[k]),
            'vx_mps': float(v_new[k]),
            'ax_mps2': float(ax_new[k]),
        }
        waypoints.append(wpnt)

    # ── Step 6: RViz Markers 생성 ──
    centerline_markers = _build_sphere_markers(
        track.x, track.y, track.z,
        r=0.0, g=0.0, b=1.0, scale=0.05,  # 파란색
    )

    raceline_markers = _build_cylinder_markers(
        x_new, y_new, z_new, v_new,
        r=1.0, g=1.0, b=0.0,  # 노란색
    )

    trackbounds_markers = _build_trackbounds_markers(track)

    # ── JSON 출력 (참고 포맷 호환) ──
    output = {
        'map_info_str': {
            'data': f'estimated lap time: {laptime:.4f}s; maximum speed: {v_new.max():.4f}m/s; '
        },
        'est_lap_time': {'data': float(laptime)},
        'centerline_markers': {'markers': centerline_markers},
        'centerline_waypoints': _build_centerline_waypoints(track),
        'global_traj_markers_iqp': {'markers': []},
        'global_traj_wpnts_iqp': {
            'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''},
            'wpnts': [],
        },
        'global_traj_markers_sp': {'markers': raceline_markers},
        'global_traj_wpnts_sp': {
            'header': {'seq': 1, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''},
            'wpnts': waypoints,
        },
        'trackbounds_markers': {'markers': trackbounds_markers},
    }

    with open(output_path, 'w') as f:
        json.dump(output, f, indent=2)

    # 검증 출력
    ds_check = np.sqrt(np.diff(x_new)**2 + np.diff(y_new)**2 + np.diff(z_new)**2)
    print(f'입력: {raceline_path}')
    print(f'출력: {output_path}')
    print(f'  waypoints: {n_new}개 (spacing={spacing}m)')
    print(f'  실제 총 경로 길이: {total_length:.3f}m')
    print(f'  laptime: {laptime:.3f}s')
    print(f'  v: [{v_new.min():.2f}, {v_new.max():.2f}] m/s')
    print(f'  z: [{z_new.min():.3f}, {z_new.max():.3f}] m')
    print(f'  s_m 간격: {np.diff(arc_new).mean():.6f} (등간격)')
    print(f'  실제 거리 간격: min={ds_check.min():.6f}, max={ds_check.max():.6f}, std={ds_check.std():.6f}')
    print(f'  markers: centerline={len(centerline_markers)}, raceline={len(raceline_markers)}, trackbounds={len(trackbounds_markers)}')
    print(f'\nSample waypoint [0]:')
    for key, val in waypoints[0].items():
        print(f'  {key}: {val}')


def _build_centerline_waypoints(track):
    """smoothed track 센터라인을 waypoint 형식으로 변환."""
    n_pts = len(track.s)
    wpnts = []
    for k in range(n_pts):
        wpnts.append({
            'id': k,
            's_m': float(track.s[k]),
            'd_m': 0.0,
            'x_m': float(track.x[k]),
            'y_m': float(track.y[k]),
            'z_m': float(track.z[k]),
            'd_right': float(abs(track.w_tr_right[k])),
            'd_left': float(abs(track.w_tr_left[k])),
            'psi_rad': float(track.theta[k]),
            'kappa_radpm': float(track.Omega_z[k]),
            'vx_mps': 0.0,
            'ax_mps2': 0.0,
        })
    return {
        'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''},
        'wpnts': wpnts,
    }


if __name__ == '__main__':
    export_waypoints()
