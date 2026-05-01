import os
import argparse
## IY : yaml for tuning override (was not imported)
import yaml
## IY : end
from casadi import *
import multiprocessing
from joblib import Parallel, delayed

### HJ : add argparse for vehicle_name (was hardcoded to 'rc_car_10th')
parser = argparse.ArgumentParser(description='Generate diamond representation for GG diagrams')
parser.add_argument('--vehicle_name', type=str, default='rc_car_10th', help='Vehicle name (default: rc_car_10th)')
## IY : add tuning CLI args (same as fast_gen_gg_diagrams.py) to enable
#       diamond post-process (gg_exp_scale, ax_max/min_scale, ay_scale).
#       --tuning_name decouples the tuning file name from --vehicle_name.
parser.add_argument('--tuning', action='store_true', default=False,
                    help='Apply tuning_<name>.yml post-process overrides')
parser.add_argument('--tuning_name', type=str, default=None,
                    help='Tuning file suffix (default: same as --vehicle_name)')
## IY : end
args, _ = parser.parse_known_args()
vehicle_name = args.vehicle_name
# paths
dir_path = os.path.dirname(os.path.abspath(__file__))
### HJ : read from fast_ggv_gen/output/ (never touch original gg_diagrams)
gg_diagram_path = os.path.join(dir_path, 'output')

num_cores = multiprocessing.cpu_count()

## IY : load diamond post-process params
#       Reads post-process keys from the UNIFIED params yml
#       (params_<vehicle_name>.yml), which contains all NLP + post-process
#       + raceline keys in one file. NLP/raceline keys are ignored here.
#       If --tuning flag is given, also tries tuning_<name>.yml as fallback
#       (backward compatibility with manual CLI usage).
#       All params default to None (= apply nothing = NLP fit as-is).
# (previous: read from tuning_<name>.yml only via --tuning flag)
tuning_post = {
    'gg_exp_scale':  None,
    'ax_max_scale':  None,
    'ax_min_scale':  None,
    'ay_scale':      None,
}

_data_path = os.path.join(dir_path, '..', 'global_line', 'data')

## IY : primary: read from params_<vehicle_name>.yml (unified yml from gg_tuner)
_params_path = os.path.join(_data_path, 'vehicle_params',
                            'params_' + vehicle_name + '.yml')
_loaded_from = None
if os.path.exists(_params_path):
    with open(_params_path, 'r') as _stream:
        _params_raw = yaml.safe_load(_stream) or {}
    for _key in tuning_post:
        if _key in _params_raw and _params_raw[_key] is not None:
            tuning_post[_key] = float(_params_raw[_key])
    _loaded_from = _params_path
## IY : end

## IY : fallback: if --tuning flag given, also check tuning_<name>.yml
#       (for backward compat with manual CLI: run_on_container.sh with TUNING=1)
if args.tuning and _loaded_from is None:
    _tuning_name = args.tuning_name if args.tuning_name else vehicle_name
    _tuning_path = os.path.join(_data_path, 'vehicle_params',
                                'tuning_' + _tuning_name + '.yml')
    if os.path.exists(_tuning_path):
        with open(_tuning_path, 'r') as _stream:
            _tuning_raw = yaml.safe_load(_stream) or {}
        for _key in tuning_post:
            if _key in _tuning_raw and _tuning_raw[_key] is not None:
                tuning_post[_key] = float(_tuning_raw[_key])
        _loaded_from = _tuning_path
    else:
        print(f'[gen_diamond] WARNING: --tuning specified but {_tuning_path} '
              f'not found')
## IY : end

_active = {k: v for k, v in tuning_post.items() if v is not None}
if _active:
    print(f'[gen_diamond] Post-process params from {_loaded_from}:')
    for _k, _v in _active.items():
        print(f'               {_k} = {_v}')
else:
    print(f'[gen_diamond] No post-process params set → NLP fit preserved as-is')
## IY : end


def gen_diamond_representation(alpha_list, rho_list):
    # diamond representation
    gg_exponent = MX.sym('gg_exponent')
    ax_min = MX.sym('ax_min')
    ax_max = MX.sym('ax_max')
    ay_max = MX.sym('ay_max')

    x = vertcat(
        gg_exponent,
        ax_min,
        ax_max,
        ay_max
    )
    lbx = vertcat(
        1.0,
        - np.interp(-np.pi / 2.0, alpha_list, rho_list),
        0.0,
        0.0
    )
    ubx = vertcat(
        2.0,
        0.0,
        np.interp(np.pi / 2.0, alpha_list, rho_list),
        np.interp(0.0, alpha_list, rho_list, 0.0)
    )

    f = 0
    g_const = []
    lbg = []
    ubg = []

    for alpha in np.linspace(
            -np.pi / 2.0,
            np.pi / 2.0,
            200,
    ):
        rho_max = np.interp(alpha, alpha_list, rho_list)

        ay = power(
            fabs(ax_min) ** gg_exponent / (
                        tan(fabs(alpha)) ** gg_exponent + (fabs(ax_min) / ay_max) ** gg_exponent),
            1.0 / gg_exponent
        )
        ax = ay * tan(fabs(alpha))
        rho_diamond = sqrt(ax ** 2 + ay ** 2)

        if alpha > 0.0:
            rho_diamond = fmin(
                rho_diamond,
                ax_max / sin(alpha)
            )

        g_const += [rho_max - rho_diamond]
        lbg += [0.0]
        ubg += [np.inf]

        f -= rho_diamond**2

    x0 = vertcat(
        1.0,
        -5.1,
        5.0,
        5.0
    )
    nlp = {"x": x, "f": f, "g": vertcat(*g_const)}
    opts = {
        "verbose": False, "ipopt.print_level": 0, "print_time": 0,
        "ipopt.hessian_approximation": 'limited-memory',
    }
    solver = nlpsol("solver", "ipopt", nlp, opts)
    x_opt = solver(x0=x0, lbx=lbx, ubx=ubx, lbg=vertcat(*lbg), ubg=vertcat(*ubg))['x']

    gg_exponent = float(x_opt[0])
    ax_min = float(x_opt[1])
    ax_max = float(x_opt[2])
    ay_max = float(x_opt[3])

    return gg_exponent, ax_min, ax_max, ay_max


def gen_diamond_representation_for_V(alpha_list, rho_list):
    gg_exponent_tmp = []
    ax_min_tmp = []
    ax_max_tmp = []
    ay_max_tmp = []

    for rho in rho_list:
        # [TEMP] skip diamond fitting for all-zero rho entries
        # caused by gen_gg_diagrams solver failing at extreme V/g combos
        # TODO: fix by improving solver scaling or narrowing g range, then remove this
        if np.all(rho == 0):
            gg_exponent_tmp.append(1.0)
            ax_min_tmp.append(0.0)
            ax_max_tmp.append(0.0)
            ay_max_tmp.append(0.0)
            continue
        gg_exponent, ax_min, ax_max, ay_max = gen_diamond_representation(
            alpha_list=alpha_list,
            rho_list=rho
        )
        gg_exponent_tmp.append(gg_exponent)
        ax_min_tmp.append(ax_min)
        ax_max_tmp.append(ax_max)
        ay_max_tmp.append(ay_max)

    return gg_exponent_tmp, ax_min_tmp, ax_max_tmp, ay_max_tmp


for frame in ['vehicle', 'velocity']:
    path = os.path.join(gg_diagram_path, vehicle_name, frame + '_frame')
    V_list = np.load(os.path.join(path, 'v_list.npy'))
    V_max = V_list.max()
    g_list = np.load(os.path.join(path, 'g_list.npy'))
    g_min = g_list.min()
    g_max = g_list.max()
    # polar coordinates
    alpha_list = np.load(os.path.join(path, 'alpha_list.npy'))
    rho_list = np.load(os.path.join(path, 'rho.npy'))

    processed_list = Parallel(
        n_jobs=num_cores
    )(
        delayed(gen_diamond_representation_for_V)(alpha_list, rho) for rho in rho_list
    )

    gg_exponent_list = [tmp[0] for tmp in processed_list]
    ax_min_list = [tmp[1] for tmp in processed_list]
    ax_max_list = [tmp[2] for tmp in processed_list]
    ay_max_list = [tmp[3] for tmp in processed_list]

    ## IY : apply diamond post-process scales (null = NLP fit as-is)
    #       - gg_exp_scale: multiplies fitted exponent, clipped to [1.0, 2.0].
    #         Preserves V/g variation while shifting toward diamond (<1) or
    #         ellipse (>1). Does NOT throw away NLP variation.
    #       - ax_max_scale / ax_min_scale: directional accel/brake scaling
    #         (can be used independently to bias brake-heavy vs accel-heavy).
    #       - ay_scale: corner capability scaling.
    #       Applied AFTER diamond fit, to both vehicle and velocity frames.
    gg_arr = np.asarray(gg_exponent_list, dtype=np.float64)
    ax_min_arr = np.asarray(ax_min_list,  dtype=np.float64)
    ax_max_arr = np.asarray(ax_max_list,  dtype=np.float64)
    ay_max_arr = np.asarray(ay_max_list,  dtype=np.float64)

    _applied_log = []
    if tuning_post['gg_exp_scale'] is not None:
        _s = tuning_post['gg_exp_scale']
        gg_before_min, gg_before_max = float(gg_arr.min()), float(gg_arr.max())
        gg_arr = np.clip(gg_arr * _s, 1.0, 2.0)
        _applied_log.append(
            f"gg_exp_scale={_s}: [{gg_before_min:.3f},{gg_before_max:.3f}] → "
            f"[{gg_arr.min():.3f},{gg_arr.max():.3f}]")
    if tuning_post['ax_max_scale'] is not None:
        _s = tuning_post['ax_max_scale']
        ax_max_arr = ax_max_arr * _s
        _applied_log.append(f"ax_max_scale={_s}")
    if tuning_post['ax_min_scale'] is not None:
        _s = tuning_post['ax_min_scale']
        ax_min_arr = ax_min_arr * _s   # ax_min is already negative
        _applied_log.append(f"ax_min_scale={_s}")
    if tuning_post['ay_scale'] is not None:
        _s = tuning_post['ay_scale']
        ay_max_arr = ay_max_arr * _s
        _applied_log.append(f"ay_scale={_s}")

    if _applied_log:
        print(f'[gen_diamond] [{frame}_frame] post-process applied: '
              + ', '.join(_applied_log))

    gg_exponent_list = gg_arr.tolist()
    ax_min_list      = ax_min_arr.tolist()
    ax_max_list      = ax_max_arr.tolist()
    ay_max_list      = ay_max_arr.tolist()
    ## IY : end

    out_path = os.path.join(gg_diagram_path, vehicle_name)
    np.save(os.path.join(out_path, frame + '_frame', "gg_exponent.npy"), gg_exponent_list)
    np.save(os.path.join(out_path, frame + '_frame', "ax_min.npy"), ax_min_list)
    np.save(os.path.join(out_path, frame + '_frame', "ax_max.npy"), ax_max_list)
    np.save(os.path.join(out_path, frame + '_frame', "ay_max.npy"), ay_max_list)

## IY : slope diamond fitting — produces 3D arrays [V_N, g_N, slope_N]
slope_data_root = os.path.join(gg_diagram_path, vehicle_name, 'slope_data')
## IY : honor enable_slope meta (skip stale slope_data/ from prior enable_slope=True run)
enable_slope_meta = True
_es_meta_path = os.path.join(gg_diagram_path, vehicle_name,
                              'velocity_frame', 'enable_slope.npy')
if os.path.isfile(_es_meta_path):
    try:
        enable_slope_meta = bool(np.load(_es_meta_path))
    except Exception:
        pass
## IY : end
if enable_slope_meta and os.path.isdir(slope_data_root):
    print(f'\n[gen_diamond] ===== Slope diamond fitting =====')
    for frame in ['vehicle', 'velocity']:
        slope_frame_dir = os.path.join(slope_data_root, frame + '_frame')
        if not os.path.isdir(slope_frame_dir):
            continue
        slope_list = np.load(os.path.join(slope_frame_dir, 'slope_list.npy'))
        slope_list_deg = np.load(os.path.join(slope_frame_dir, 'slope_list_deg.npy'))
        alpha_list = np.load(os.path.join(slope_frame_dir, 'alpha_list.npy'))
        rho_4d = np.load(os.path.join(slope_frame_dir, 'rho.npy'))
        # rho_4d shape: [slope_N, V_N, g_N, alpha_N]
        slope_N, V_N_s, g_N_s, _ = rho_4d.shape

        print(f'[gen_diamond] [{frame}_frame] slope_N={slope_N}, '
              f'V_N={V_N_s}, g_N={g_N_s}')

        # Fit diamond for each slope angle
        # Output shape: [V_N, g_N, slope_N]
        gg_exp_3d = np.zeros((V_N_s, g_N_s, slope_N))
        ax_min_3d = np.zeros((V_N_s, g_N_s, slope_N))
        ax_max_3d = np.zeros((V_N_s, g_N_s, slope_N))
        ay_max_3d = np.zeros((V_N_s, g_N_s, slope_N))

        for si in range(slope_N):
            # rho_4d[si] shape: [V_N, g_N, alpha_N] — same as standard rho
            rho_this_slope = rho_4d[si]
            results = Parallel(n_jobs=num_cores)(
                delayed(gen_diamond_representation_for_V)(alpha_list, rho_v)
                for rho_v in rho_this_slope
            )
            for vi, (gg_e, ax_mn, ax_mx, ay_mx) in enumerate(results):
                for gi in range(g_N_s):
                    gg_exp_3d[vi, gi, si] = gg_e[gi]
                    ax_min_3d[vi, gi, si] = ax_mn[gi]
                    ax_max_3d[vi, gi, si] = ax_mx[gi]
                    ay_max_3d[vi, gi, si] = ay_mx[gi]
            print(f'[gen_diamond] slope {slope_list_deg[si]:+.1f}°: fitted')

        # Apply post-process scales
        if tuning_post['gg_exp_scale'] is not None:
            gg_exp_3d = np.clip(gg_exp_3d * tuning_post['gg_exp_scale'], 1.0, 2.0)
        if tuning_post['ax_max_scale'] is not None:
            ax_max_3d *= tuning_post['ax_max_scale']
        if tuning_post['ax_min_scale'] is not None:
            ax_min_3d *= tuning_post['ax_min_scale']
        if tuning_post['ay_scale'] is not None:
            ay_max_3d *= tuning_post['ay_scale']

        # Save 3D diamond arrays to the main frame directory
        main_frame_dir = os.path.join(gg_diagram_path, vehicle_name, frame + '_frame')
        np.save(os.path.join(main_frame_dir, 'slope_list.npy'), slope_list)
        np.save(os.path.join(main_frame_dir, 'slope_list_deg.npy'), slope_list_deg)
        np.save(os.path.join(main_frame_dir, 'gg_exponent_3d.npy'), gg_exp_3d)
        np.save(os.path.join(main_frame_dir, 'ax_min_3d.npy'), ax_min_3d)
        np.save(os.path.join(main_frame_dir, 'ax_max_3d.npy'), ax_max_3d)
        np.save(os.path.join(main_frame_dir, 'ay_max_3d.npy'), ay_max_3d)
        print(f'[gen_diamond] [{frame}_frame] 3D diamond saved: '
              f'{gg_exp_3d.shape}')
elif not enable_slope_meta:
    print(f'[gen_diamond] enable_slope=False meta → skip 3D diamond fitting')
else:
    print(f'[gen_diamond] No slope_data/ found — 2D only')
## IY : end

# EOF
