#!/usr/bin/env python3

import os
import sys
import copy
import json
import shutil
import struct
import subprocess
import threading
import time
from datetime import datetime

import numpy as np
import rospy
import yaml
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from stack_master.cfg import GGTunerConfig
## IY : Trigger service for FBGA hot-reload
from std_srvs.srv import Trigger
## IY : end


class GGTunerNode:

    TIRE_KEYS = ['lambda_mu_x', 'lambda_mu_y', 'p_Dx_2', 'p_Dy_2', 'friction']
    VEHICLE_KEYS = ['P_max', 'v_max', 'epsilon',
                    'P_brake_max', 'ax_max_cap', 'ax_min_cap', 'ay_max_cap']
    CAP_KEYS = ['P_brake_max', 'ax_max_cap', 'ax_min_cap', 'ay_max_cap']
    POST_KEYS = ['gg_exp_scale', 'ax_max_scale', 'ax_min_scale', 'ay_scale']
    RACELINE_KEYS = ['V_min', 'safety_distance', 'w_T', 'w_jx', 'w_jy', 'w_dOmega_z']
    ALL_TUNING_KEYS = TIRE_KEYS + VEHICLE_KEYS + POST_KEYS + RACELINE_KEYS
    ## IY : end

    def __init__(self):
        rospy.loginfo("[GGTuner] Initializing...")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        race_stack_root = os.path.dirname(os.path.dirname(script_dir))
        self.race_stack_root = race_stack_root

        self.data_path = os.path.join(
            race_stack_root, 'planner', '3d_gb_optimizer', 'global_line', 'data')
        self.maps_dir = os.path.join(race_stack_root, 'stack_master', 'maps')

        self.fast_ggv_dir = os.path.join(
            race_stack_root, 'planner', '3d_gb_optimizer', 'fast_ggv_gen')
        self.fast_ggv_script = os.path.join(self.fast_ggv_dir, 'run_on_container.sh')
        self.fast_ggv_output_dir = os.path.join(self.fast_ggv_dir, 'output')

        self.base_vehicle = rospy.get_param("~base_vehicle", "rc_car_10th")

        base_yml_path = os.path.join(
            self.data_path, 'vehicle_params',
            'params_' + self.base_vehicle + '.yml')
        backup_yml_path = os.path.join(
            self.data_path, 'vehicle_params',
            'params_' + self.base_vehicle + '_backup.yml')

        loaded_path = None
        for path in [base_yml_path, backup_yml_path]:
            if os.path.exists(path) and not os.path.islink(path) or \
               (os.path.islink(path) and os.path.exists(os.readlink(path) if not os.path.isabs(os.readlink(path)) else path)):
                try:
                    with open(path, 'r') as f:
                        self.base_params = yaml.safe_load(f)
                    loaded_path = path
                    break
                except (yaml.YAMLError, OSError):
                    continue
        if loaded_path is None:
            rospy.logerr(f"[GGTuner] Base params not found: tried {base_yml_path} and {backup_yml_path}")
            raise FileNotFoundError(base_yml_path)
        rospy.loginfo(f"[GGTuner] Base params loaded: {loaded_path}")
        ## IY : end

        ## IY : scan available maps
        self.available_maps = self._scan_maps_dir()
        rospy.loginfo(f"[GGTuner] Available maps ({len(self.available_maps)}): "
                      f"{self.available_maps}")

        if not os.path.exists(self.fast_ggv_script):
            rospy.logwarn(f"[GGTuner] fast_ggv script missing: {self.fast_ggv_script}")
        ## IY : end

        self.status_pub = rospy.Publisher(
            '/gg_compute_status', String, queue_size=5, latch=True)
        self.status_pub.publish(f"READY: {self.base_vehicle}")

        self.pipeline_thread = None
        self.pipeline_lock = threading.Lock()
        self.fbga_proc = None

        rospy.on_shutdown(self._shutdown_cleanup)

        self.srv = Server(GGTunerConfig, self.reconfigure_cb)
        rospy.loginfo("[GGTuner] Ready. Use rqt_reconfigure → /gg_tuner")

    def _round_tuning(self, tuning_dict):
        return {k: round(v, 4) for k, v in sorted(tuning_dict.items())}

    def _find_cached(self, tuning_dict):
        rounded = self._round_tuning(tuning_dict)
        gg_dir = os.path.join(self.data_path, 'gg_diagrams')
        prefix = self.base_vehicle + '_v'
        if not os.path.exists(gg_dir):
            return None
        for name in sorted(os.listdir(gg_dir)):
            if not name.startswith(prefix):
                continue
            full = os.path.join(gg_dir, name)
            if os.path.islink(full):
                continue
            suffix = name[len(prefix):]
            if not suffix.isdigit():
                continue
            meta_path = os.path.join(full, 'params_used.json')
            if not os.path.exists(meta_path):
                continue
            try:
                with open(meta_path, 'r') as f:
                    meta = json.load(f)
                saved = {k: round(v, 4) for k, v in sorted(meta['tuning'].items())}
                if saved == rounded:
                    return name
            except (json.JSONDecodeError, KeyError):
                continue
        return None

    def _next_version(self):
        gg_dir = os.path.join(self.data_path, 'gg_diagrams')
        prefix = self.base_vehicle + '_v'
        max_ver = 0
        if os.path.exists(gg_dir):
            for name in os.listdir(gg_dir):
                if not name.startswith(prefix):
                    continue
                suffix = name[len(prefix):]
                if not suffix.isdigit():
                    continue
                max_ver = max(max_ver, int(suffix))
        return max_ver + 1
    
    def _merge_all_params(self, tuning_dict):
        merged = copy.deepcopy(self.base_params)
        # NLP: tire
        for key in self.TIRE_KEYS:
            if key in tuning_dict:
                if key == 'friction':
                    fric_val = float(tuning_dict[key])
                    merged['tire_params']['p_Dx_1'] = fric_val
                    merged['tire_params']['p_Dy_1'] = fric_val
                    continue
                merged['tire_params'][key] = tuning_dict[key]
        # NLP: vehicle (caps: 0.0 → None)
        for key in self.VEHICLE_KEYS:
            if key not in tuning_dict:
                continue
            val = tuning_dict[key]
            if key in self.CAP_KEYS and float(val) <= 0.0:
                merged['vehicle_params'][key] = None
            else:
                merged['vehicle_params'][key] = val
        # Post-process + raceline → top-level
        for key in self.POST_KEYS + self.RACELINE_KEYS:
            if key in tuning_dict:
                merged[key] = float(tuning_dict[key])
        return merged

    def _save_params_yml(self, vehicle_name, merged_params):
        yml_path = os.path.join(
            self.data_path, 'vehicle_params',
            'params_' + vehicle_name + '.yml')
        with open(yml_path, 'w') as f:
            yaml.dump(merged_params, f, default_flow_style=False, allow_unicode=True)
        ## IY : copy to latest (real file, not symlink)
        latest_path = os.path.join(
            self.data_path, 'vehicle_params',
            'params_' + self.base_vehicle + '_latest.yml')
        shutil.copy2(yml_path, latest_path)
        ## IY : end
        rospy.loginfo(f"[GGTuner] Params saved: {yml_path} → latest copied")
        return yml_path

    def _save_meta(self, vehicle_name, tuning_dict):
        out_dir = os.path.join(self.data_path, 'gg_diagrams', vehicle_name)
        os.makedirs(out_dir, exist_ok=True)
        meta = {
            'vehicle_name': vehicle_name,
            'base_vehicle': self.base_vehicle,
            'tuning': tuning_dict,
            'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        }
        meta_path = os.path.join(out_dir, 'params_used.json')
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)
        rospy.loginfo(f"[GGTuner] Meta saved: {meta_path}")

    def _write_latest_params_yml(self, merged_params):
        latest_yml = os.path.join(
            self.data_path, 'vehicle_params',
            f'params_{self.base_vehicle}_latest.yml')
        with open(latest_yml, 'w') as f:
            yaml.dump(merged_params, f, default_flow_style=False,
                      allow_unicode=True)
        rospy.loginfo(f"[GGTuner] latest yml written: {latest_yml}")
        return latest_yml

    def _restore_to_latest(self, cached_name):
        latest_name = f'{self.base_vehicle}_latest'
        src_gg = os.path.join(self.data_path, 'gg_diagrams', cached_name)
        dst_gg = os.path.join(self.data_path, 'gg_diagrams', latest_name)
        src_yml = os.path.join(self.data_path, 'vehicle_params',
                               f'params_{cached_name}.yml')
        dst_yml = os.path.join(self.data_path, 'vehicle_params',
                               f'params_{latest_name}.yml')
        if not os.path.exists(src_gg):
            rospy.logerr(f"[GGTuner] cached gg_diagrams missing: {src_gg}")
            return False
        if os.path.islink(dst_gg):
            os.unlink(dst_gg)
        elif os.path.exists(dst_gg):
            shutil.rmtree(dst_gg)
        shutil.copytree(src_gg, dst_gg)
        if os.path.exists(src_yml):
            shutil.copy2(src_yml, dst_yml)
        rospy.loginfo(f"[GGTuner] restored {cached_name} → latest")
        return True

    def _snapshot_latest_to_version(self, tuning_dict=None):
        map_name = rospy.get_param('/map', '')
        sectors = self._read_friction_sectors(map_name)
        n_sec = len(sectors)

        ver = self._next_version()
        version_prefix = f'{self.base_vehicle}_v{ver}'
        rospy.loginfo(f"[GGTuner] ===== SAVE snapshot: {version_prefix} =====")
        self.status_pub.publish(f"SAVING: {version_prefix}")

        saved_frictions = set()  # dedup tracker

        if n_sec > 0:
            # Per-sector latest → v<N>_f<NNN> (unique friction dedup)
            for i in range(min(n_sec, 5)):
                sec_name = f'{self.base_vehicle}_latest_sec{i}'
                sec_gg = os.path.join(
                    self.data_path, 'gg_diagrams', sec_name)
                sec_yml = os.path.join(
                    self.data_path, 'vehicle_params',
                    f'params_{sec_name}.yml')
                if not os.path.exists(sec_gg):
                    rospy.logwarn(
                        f"[GGTuner] SAVE: latest_sec{i} missing → skip")
                    continue
                fric = float(sectors[i]['friction'])
                fric_int = int(round(fric * 100))
                if fric_int in saved_frictions:
                    rospy.loginfo(
                        f"[GGTuner] SAVE: sec{i} (f={fric:.3f}) already saved "
                        f"as f{fric_int:03d} → skip")
                    continue
                snapshot_name = f'{version_prefix}_f{fric_int:03d}'
                dst_gg = os.path.join(
                    self.data_path, 'gg_diagrams', snapshot_name)
                if os.path.exists(dst_gg):
                    shutil.rmtree(dst_gg)
                real_src = (os.path.realpath(sec_gg)
                            if os.path.islink(sec_gg) else sec_gg)
                shutil.copytree(real_src, dst_gg)
                if os.path.exists(sec_yml):
                    dst_yml = os.path.join(
                        self.data_path, 'vehicle_params',
                        f'params_{snapshot_name}.yml')
                    shutil.copy2(sec_yml, dst_yml)
                # meta
                meta = {
                    'vehicle_name': snapshot_name,
                    'base_vehicle': self.base_vehicle,
                    'friction': fric,
                    'sector_index_source': i,
                    'tuning': tuning_dict or {},
                    'saved_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                }
                meta_path = os.path.join(dst_gg, 'params_used.json')
                with open(meta_path, 'w') as f:
                    json.dump(meta, f, indent=2, ensure_ascii=False)
                saved_frictions.add(fric_int)
                rospy.loginfo(f"[GGTuner] SAVED: {snapshot_name}")

            if not saved_frictions:
                rospy.logerr("[GGTuner] SAVE failed: no latest_sec<i> found")
                self.status_pub.publish("SAVE_FAILED: no latest_sec")
                return False

            rospy.loginfo(
                f"[GGTuner] SAVE done: {version_prefix} "
                f"({len(saved_frictions)} unique frictions)")
            self.status_pub.publish(f"SAVED: {version_prefix}")
            return True

        # Fallback: no sectors → legacy single-latest save
        latest_name = f'{self.base_vehicle}_latest'
        latest_gg = os.path.join(self.data_path, 'gg_diagrams', latest_name)
        latest_yml = os.path.join(self.data_path, 'vehicle_params',
                                  f'params_{latest_name}.yml')
        if not os.path.exists(latest_gg):
            rospy.logerr(f"[GGTuner] SAVE failed: {latest_gg} missing")
            self.status_pub.publish("SAVE_FAILED: no latest gg")
            return False
        if not os.path.exists(latest_yml):
            rospy.logerr(f"[GGTuner] SAVE failed: {latest_yml} missing")
            self.status_pub.publish("SAVE_FAILED: no latest yml")
            return False
        snapshot_name = version_prefix  # no friction tag when no sectors
        dst_gg = os.path.join(self.data_path, 'gg_diagrams', snapshot_name)
        real_src = (os.path.realpath(latest_gg)
                    if os.path.islink(latest_gg) else latest_gg)
        shutil.copytree(real_src, dst_gg)
        dst_yml = os.path.join(self.data_path, 'vehicle_params',
                               f'params_{snapshot_name}.yml')
        shutil.copy2(latest_yml, dst_yml)

        latest_meta_path = os.path.join(latest_gg, 'params_used.json')
        meta = None
        if os.path.exists(latest_meta_path):
            try:
                with open(latest_meta_path) as f:
                    meta = json.load(f)
            except (json.JSONDecodeError, OSError):
                meta = None
        if meta is None:
            meta = {
                'vehicle_name': snapshot_name,
                'base_vehicle': self.base_vehicle,
                'tuning': tuning_dict or {},
                'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            }
        meta['vehicle_name'] = snapshot_name
        meta['saved_at'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        meta_path = os.path.join(dst_gg, 'params_used.json')
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        ## IY(0416) : also snapshot friction-variant GGV directories
        gg_parent = os.path.join(self.data_path, 'gg_diagrams')
        fric_prefix = f'{latest_name}_f'
        for name in os.listdir(gg_parent):
            if not name.startswith(fric_prefix):
                continue
            fric_suffix = name[len(latest_name):]
            src_fric_gg = os.path.join(gg_parent, name)
            dst_fric_gg = os.path.join(gg_parent, f'{snapshot_name}{fric_suffix}')
            if os.path.isdir(src_fric_gg) and not os.path.islink(src_fric_gg):
                shutil.copytree(src_fric_gg, dst_fric_gg)
                src_fric_yml = os.path.join(
                    self.data_path, 'vehicle_params', f'params_{name}.yml')
                if os.path.exists(src_fric_yml):
                    dst_fric_yml = os.path.join(
                        self.data_path, 'vehicle_params',
                        f'params_{snapshot_name}{fric_suffix}.yml')
                    shutil.copy2(src_fric_yml, dst_fric_yml)
                rospy.loginfo(
                    f"[GGTuner] SAVED friction variant: {snapshot_name}{fric_suffix}")
        ## IY(0416) : end

        rospy.loginfo(f"[GGTuner] SAVED: {snapshot_name}")
        self.status_pub.publish(f"SAVED: {snapshot_name}")
        return True
    ## IY : end

    def _scan_maps_dir(self):
        if not os.path.exists(self.maps_dir):
            return []
        return sorted([n for n in os.listdir(self.maps_dir)
                       if os.path.isdir(os.path.join(self.maps_dir, n))])

    def _generate_gg_bin(self, npy_dir):
        bin_path = os.path.join(npy_dir, 'gg.bin')
        try:
            v_list = np.load(os.path.join(npy_dir, 'v_list.npy')).astype(np.float64)
            g_list = np.load(os.path.join(npy_dir, 'g_list.npy')).astype(np.float64)
            ax_max = np.load(os.path.join(npy_dir, 'ax_max.npy')).astype(np.float64)
            ax_min = np.load(os.path.join(npy_dir, 'ax_min.npy')).astype(np.float64)
            ay_max = np.load(os.path.join(npy_dir, 'ay_max.npy')).astype(np.float64)
            gg_exp = np.load(os.path.join(npy_dir, 'gg_exponent.npy')).astype(np.float64)
        except (OSError, FileNotFoundError) as e:
            rospy.logerr(f"[GGTuner] gg.bin gen failed (npy missing): {e}")
            return False
        nv, ng = len(v_list), len(g_list)
        try:
            with open(bin_path, 'wb') as f:
                f.write(struct.pack('II', nv, ng))
                for arr in [v_list, g_list, ax_max, ax_min, ay_max, gg_exp]:
                    arr.tofile(f)
        except OSError as e:
            rospy.logerr(f"[GGTuner] gg.bin write failed: {e}")
            return False
        rospy.loginfo(
            f"[GGTuner] gg.bin written: {bin_path} (nv={nv}, ng={ng}, "
            f"size={os.path.getsize(bin_path)} B)")
        return True

    def _copy_to_gg_diagrams(self, vehicle_name):
        src = os.path.join(self.fast_ggv_output_dir, vehicle_name)
        dst = os.path.join(self.data_path, 'gg_diagrams', vehicle_name)
        if not os.path.exists(src):
            rospy.logerr(f"[GGTuner] fast_ggv output missing: {src}")
            return False
        meta_backup = None
        meta_path = os.path.join(dst, 'params_used.json')
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r') as f:
                    meta_backup = f.read()
            except OSError:
                pass
        if os.path.exists(dst) or os.path.islink(dst):
            if os.path.islink(dst):
                os.unlink(dst)
            else:
                shutil.rmtree(dst)
        shutil.copytree(src, dst)
        if meta_backup is not None:
            with open(meta_path, 'w') as f:
                f.write(meta_backup)
        rospy.loginfo(f"[GGTuner] Copied: {vehicle_name} → gg_diagrams/")
        for frame in ('velocity_frame', 'vehicle_frame'):
            frame_dir = os.path.join(dst, frame)
            if os.path.isdir(frame_dir):
                self._generate_gg_bin(frame_dir)
        return True

    ## IY : symlinks for DIRECTORIES only (params latest is a real file copy)
    def _update_dir_symlinks(self, vehicle_name):
        latest_name = f'{self.base_vehicle}_latest'
        targets = [
            (self.fast_ggv_output_dir, latest_name, vehicle_name),
            (os.path.join(self.data_path, 'gg_diagrams'), latest_name, vehicle_name),
        ]
        for parent, link_name, target in targets:
            link_path = os.path.join(parent, link_name)
            target_path = os.path.join(parent, target)
            if not os.path.exists(target_path):
                continue
            try:
                if os.path.islink(link_path) or os.path.exists(link_path):
                    os.unlink(link_path)
                os.symlink(target, link_path)
                rospy.loginfo(f"[GGTuner] symlink: {link_name} → {target}")
            except OSError as e:
                rospy.logwarn(f"[GGTuner] symlink failed: {e}")
    ## IY : end

    def _run_and_stream(self, cmd, tag, timeout=600, env=None):
        rospy.loginfo(f"[GGTuner] [{tag}] cmd: {' '.join(cmd)}")
        run_env = None
        if env is not None:
            run_env = os.environ.copy()
            run_env.update(env)
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, env=run_env)
        except (FileNotFoundError, OSError) as e:
            rospy.logerr(f"[GGTuner] [{tag}] Popen failed: {e}")
            return False
        start_time = time.time()
        try:
            for line in iter(proc.stdout.readline, ''):
                if time.time() - start_time > timeout:
                    rospy.logerr(f"[GGTuner] [{tag}] timeout ({timeout}s)")
                    proc.kill()
                    return False
                line = line.rstrip()
                if line:
                    rospy.loginfo(f"[{tag}] {line}")
        except Exception as e:
            rospy.logerr(f"[GGTuner] [{tag}] stream error: {e}")
            proc.kill()
            return False
        proc.wait()
        ok = (proc.returncode == 0)
        if ok:
            rospy.loginfo(f"[GGTuner] [{tag}] done (rc=0)")
        else:
            rospy.logerr(f"[GGTuner] [{tag}] failed (rc={proc.returncode})")
        return ok

    ## IY : fast_ggv — no --tuning, unified params yml already has everything
    def _run_fast_ggv(self, vehicle_name, full_resolution=False):
        rospy.loginfo(f"[GGTuner] [fast_ggv] starting: {vehicle_name}")
        self.status_pub.publish(f"GGV_COMPUTING: {vehicle_name}")
        if not os.path.exists(self.fast_ggv_script):
            rospy.logerr(f"[GGTuner] fast_ggv script missing: {self.fast_ggv_script}")
            return False
        resolution = '--full' if full_resolution else '--fast'
        cmd = ['bash', self.fast_ggv_script, vehicle_name, resolution]
        return self._run_and_stream(cmd, tag='fast_ggv', timeout=600)
    ## IY : end

    def _read_friction_sectors(self, map_name):
        """Read friction sectors — rosparam first (live rqt values), yaml fallback."""
        # 1) try rosparam (set by friction_sector_server rqt)
        try:
            n_sec = rospy.get_param('/friction_map_params/n_sectors', 0)
            if n_sec > 0:
                sectors = []
                for i in range(n_sec):
                    sectors.append({
                        'start': int(rospy.get_param(f'/friction_map_params/Sector{i}/start', 0)),
                        'end':   int(rospy.get_param(f'/friction_map_params/Sector{i}/end', 0)),
                        'friction': float(rospy.get_param(f'/friction_map_params/Sector{i}/friction', -1.0)),
                    })
                valid = [s for s in sectors if s['friction'] > 0]
                if valid:
                    rospy.loginfo(f"[GGTuner] Friction sectors from rosparam: {len(valid)} sectors")
                    return valid
        except Exception:
            pass
        # 2) fallback: read yaml file
        yaml_path = os.path.join(self.maps_dir, map_name, 'friction_scaling.yaml')
        if not os.path.exists(yaml_path):
            rospy.loginfo(f"[GGTuner] No friction sectors for {map_name}")
            return []
        try:
            with open(yaml_path) as f:
                data = yaml.safe_load(f)
            sectors = []
            for i in range(data.get('n_sectors', 0)):
                sec = data.get(f'Sector{i}', {})
                fric = sec.get('friction', -1.0)
                if fric > 0:
                    sectors.append({'start': sec.get('start', 0),
                                    'end': sec.get('end', 0),
                                    'friction': float(fric)})
            rospy.loginfo(f"[GGTuner] Friction sectors from yaml: {len(sectors)} sectors")
            return sectors
        except Exception as e:
            rospy.logwarn(f"[GGTuner] friction_scaling.yaml parse error: {e}")
            return []


    def _replace_dir(self, dst, src):
        if os.path.islink(dst):
            os.unlink(dst)
        elif os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)

    def _generate_sector_ggvs(self, vehicle_name, sectors, slot_overrides,
                               full_resolution, run_ggv):

        n_sec = len(sectors)
        if n_sec == 0:
            rospy.loginfo("[GGTuner] No sectors → skip sector-GGVs")
            return True, "no_sectors"

        # Warn about slots beyond n_sectors
        for i in range(min(n_sec, 5), 5):
            slot = (slot_overrides.get(i, '') or '').strip()
            if slot:
                rospy.logwarn(
                    f"[GGTuner] ggv_sector{i}='{slot}' but n_sectors={n_sec} "
                    f"→ slot ignored")

        # Load base merged yml (for fresh-calc friction variants)
        base_yml = os.path.join(
            self.data_path, 'vehicle_params', f'params_{vehicle_name}.yml')
        if not os.path.exists(base_yml):
            rospy.logerr(f"[GGTuner] base yml not found: {base_yml}")
            return False, "base_yml_missing"
        with open(base_yml) as f:
            base_merged = yaml.safe_load(f)

        fresh_cache = {}  # friction(float) → source vehicle_name (work dir)
        workdir_names = []  # track for cleanup

        ## IY : pre-populate cache with Stage 2's base latest to avoid recomputing
        #       the same friction (base_friction == a sector friction).
        base_friction = float(
            base_merged.get('tire_params', {}).get('p_Dx_1', 0.56))
        base_latest_gg = os.path.join(
            self.data_path, 'gg_diagrams', vehicle_name)
        if run_ggv and os.path.exists(base_latest_gg):
            fresh_cache[base_friction] = vehicle_name
            rospy.loginfo(
                f"[GGTuner] cache pre-populate: f={base_friction:.3f} "
                f"-> {vehicle_name} (skip re-compute)")
        ## IY : end

        for i in range(min(n_sec, 5)):
            sector = sectors[i]
            fric = float(sector['friction'])
            target_name = f"{self.base_vehicle}_latest_sec{i}"
            target_gg = os.path.join(
                self.data_path, 'gg_diagrams', target_name)
            target_yml = os.path.join(
                self.data_path, 'vehicle_params', f'params_{target_name}.yml')
            slot = (slot_overrides.get(i, '') or '').strip()

            if slot:
                # Restore from snapshot (always, regardless of run_ggv)
                src_gg = os.path.join(self.data_path, 'gg_diagrams', slot)
                src_yml = os.path.join(
                    self.data_path, 'vehicle_params', f'params_{slot}.yml')
                if not os.path.exists(src_gg):
                    rospy.logerr(
                        f"[GGTuner] sec{i} slot '{slot}' not found → FAIL")
                    return False, f"slot_missing:{slot}"
                self._replace_dir(target_gg, src_gg)
                if os.path.exists(src_yml):
                    shutil.copy2(src_yml, target_yml)
                rospy.loginfo(
                    f"[GGTuner] sec{i} (f={fric:.3f}): restored from {slot}")
                continue

            # Empty slot
            if not run_ggv:
                if not os.path.exists(target_gg):
                    rospy.logerr(
                        f"[GGTuner] sec{i}: latest_sec{i} missing and "
                        f"run_ggv=False → FAIL")
                    return False, f"sec{i}_no_existing"
                rospy.loginfo(
                    f"[GGTuner] sec{i} (f={fric:.3f}): reusing existing "
                    f"latest_sec{i}")
                continue

            # run_ggv=True → fresh calc (dedup per unique friction)
            if fric not in fresh_cache:
                fric_int = int(round(fric * 100))
                work_vehicle = f"{self.base_vehicle}_workf{fric_int:03d}"
                fric_params = copy.deepcopy(base_merged)
                fric_params.setdefault('tire_params', {})
                fric_params['tire_params']['p_Dx_1'] = fric
                fric_params['tire_params']['p_Dy_1'] = fric
                work_yml = os.path.join(
                    self.data_path, 'vehicle_params',
                    f'params_{work_vehicle}.yml')
                with open(work_yml, 'w') as f:
                    yaml.dump(fric_params, f, default_flow_style=False,
                              allow_unicode=True)
                rospy.loginfo(
                    f"[GGTuner] sec{i} (f={fric:.3f}): fresh calc via "
                    f"{work_vehicle}")
                if not self._run_fast_ggv(work_vehicle, full_resolution):
                    return False, f"fast_ggv_failed:{work_vehicle}"
                if not self._copy_to_gg_diagrams(work_vehicle):
                    return False, f"copy_failed:{work_vehicle}"
                fresh_cache[fric] = work_vehicle
                workdir_names.append(work_vehicle)
            else:
                rospy.loginfo(
                    f"[GGTuner] sec{i} (f={fric:.3f}): reuse cached fresh "
                    f"{fresh_cache[fric]}")

            # Copy cached fresh work dir → latest_sec<i>
            src_name = fresh_cache[fric]
            src_gg = os.path.join(self.data_path, 'gg_diagrams', src_name)
            src_yml = os.path.join(
                self.data_path, 'vehicle_params', f'params_{src_name}.yml')
            self._replace_dir(target_gg, src_gg)
            if os.path.exists(src_yml):
                shutil.copy2(src_yml, target_yml)

        # Cleanup intermediate work dirs
        for work_name in workdir_names:
            for path in [
                os.path.join(self.data_path, 'gg_diagrams', work_name),
                os.path.join(self.data_path, 'vehicle_params',
                             f'params_{work_name}.yml')]:
                try:
                    if os.path.isdir(path):
                        shutil.rmtree(path)
                    elif os.path.isfile(path):
                        os.remove(path)
                except OSError:
                    pass

        return True, "ok"
    ## IY : end

    ## IY : raceline — passes safety_distance from rqt
    def _run_raceline(self, vehicle_name, map_name, safety_distance=0.20):
        if map_name not in self.available_maps:
            rospy.logerr(f"[GGTuner] map '{map_name}' not found. "
                         f"Available: {self.available_maps}")
            self.status_pub.publish(f"FAILED_RACELINE: invalid map")
            return False
        self.status_pub.publish(f"RACELINE_STARTED: {vehicle_name}")
        rospy.loginfo(f"[GGTuner] [raceline] map={map_name}, vehicle={vehicle_name}, "
                      f"safety_distance={safety_distance}")
        cmd = [
            'roslaunch', 'stack_master', '3d_global_line.launch',
            f'map:={map_name}',
            f'vehicle:={vehicle_name}',
            f'safety_distance:={safety_distance}',
            'start_from:=5',
        ]
        ok = self._run_and_stream(cmd, tag='raceline', timeout=900)
        if ok:
            self.status_pub.publish(f"RACELINE_DONE: {vehicle_name}")
        else:
            self.status_pub.publish(f"FAILED_RACELINE: {vehicle_name}")
        return ok
    ## IY : end

    def _run_fbga_planner(self, vehicle_name, enable_mu=True, force_restart=False):
        rospy.loginfo(
            f"[GGTuner] [fbga] update: {vehicle_name}, enable_mu={enable_mu}, "
            f"force_restart={force_restart}")
        self.status_pub.publish(f"FBGA_STARTED: {vehicle_name}")

        gg_bin = os.path.join(
            self.data_path, 'gg_diagrams', vehicle_name,
            'velocity_frame', 'gg.bin')
        params_yml = os.path.join(
            self.data_path, 'vehicle_params',
            'params_' + vehicle_name + '.yml')
        if not os.path.exists(params_yml):
            rospy.logerr(f"[GGTuner] params yml missing: {params_yml}")
            self.status_pub.publish(f"FAILED_FBGA: {vehicle_name}")
            return False

        rospy.set_param('/fbga_planner/gg_bin', gg_bin)
        rospy.set_param('/fbga_planner/params_yml', params_yml)
        rospy.set_param('/fbga_planner/enable_mu', bool(enable_mu))

        if not force_restart:
            try:
                rospy.wait_for_service('/fbga/reload', timeout=0.5)
                reload_srv = rospy.ServiceProxy('/fbga/reload', Trigger)
                resp = reload_srv()
                if resp.success:
                    rospy.loginfo(
                        f"[GGTuner] [fbga] hot-reloaded: {resp.message}")
                    self.status_pub.publish(f"FBGA_DONE: {vehicle_name}")
                    return True
                else:
                    rospy.logwarn(
                        f"[GGTuner] [fbga] reload returned failure: "
                        f"{resp.message} → cold start")
            except (rospy.ROSException, rospy.ServiceException) as e:
                rospy.loginfo(
                    f"[GGTuner] [fbga] reload unavailable ({e}) → cold start")

        try:
            subprocess.run(['rosnode', 'kill', '/fbga_planner'],
                           capture_output=True, timeout=5, check=False)
            time.sleep(0.3)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        if self.fbga_proc is not None and self.fbga_proc.poll() is None:
            try:
                self.fbga_proc.terminate()
                self.fbga_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.fbga_proc.kill()
        self.fbga_proc = None

        cmd = [
            'rosrun', 'stack_master', 'fbga_velocity_planner.py',
            '__name:=fbga_planner',
            f'_gg_bin:={gg_bin}',
            f'_params_yml:={params_yml}',
            f'_enable_mu:={str(enable_mu).lower()}',
        ]
        rospy.loginfo(f"[GGTuner] [fbga] cold start: {' '.join(cmd)}")
        try:
            self.fbga_proc = subprocess.Popen(cmd)
            return True
        except OSError as e:
            rospy.logerr(f"[GGTuner] Failed to start FBGA: {e}")
            self.status_pub.publish(f"FAILED_FBGA: {vehicle_name}")
            return False

    def _kill_fbga_planner(self):
        rospy.loginfo("[GGTuner] [fbga] kill requested (checkbox off)")
        try:
            subprocess.run(['rosnode', 'kill', '/fbga_planner'],
                           capture_output=True, timeout=5, check=False)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        if self.fbga_proc is not None and self.fbga_proc.poll() is None:
            try:
                self.fbga_proc.terminate()
                self.fbga_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.fbga_proc.kill()
        self.fbga_proc = None
        rospy.loginfo("[GGTuner] [fbga] killed")
    ## IY : end

    def _run_full_pipeline(self, tuning, run_opts):
        with self.pipeline_lock:
            try:
                rospy.loginfo(f"[GGTuner] ===== Pipeline start =====")
                rospy.loginfo(f"[GGTuner] tuning: {tuning}")
                rospy.loginfo(f"[GGTuner] options: {run_opts}")
                self.status_pub.publish(f"STARTED: {self.base_vehicle}")

                vehicle_name = f"{self.base_vehicle}_latest"

                if not run_opts['run_ggv']:
                    latest_gg = os.path.join(
                        self.data_path, 'gg_diagrams', vehicle_name)
                    if not os.path.exists(latest_gg):
                        rospy.logerr(
                            f"[GGTuner] latest gg_diagrams missing: {latest_gg} "
                            f"(run_ggv=False)")
                        self.status_pub.publish("FAILED: no latest")
                        return
                    rospy.loginfo(
                        f"[GGTuner] Stage 2 SKIP (run_ggv=False), reusing latest")
                    self.status_pub.publish(f"GGV_SKIP: {vehicle_name}")
                else:
                    rospy.loginfo(
                        f"[GGTuner] Stage 2: compute fresh into latest "
                        f"(friction={tuning.get('friction', '?')})")
                    merged = self._merge_all_params(tuning)
                    self._write_latest_params_yml(merged)
                    ok = self._run_fast_ggv(
                        vehicle_name,
                        full_resolution=run_opts['full_resolution'])
                    if not ok:
                        self.status_pub.publish(f"FAILED_GGV: {vehicle_name}")
                        return
                    if not self._copy_to_gg_diagrams(vehicle_name):
                        self.status_pub.publish(f"FAILED_GGV: {vehicle_name}")
                        return
                    self._save_meta(vehicle_name, tuning)
                    self.status_pub.publish(f"GGV_DONE: {vehicle_name}")
                ## IY : end
                    
                slot_overrides = {
                    i: run_opts[f'ggv_sector{i}'] for i in range(5)
                }
                friction_sectors = self._read_friction_sectors(run_opts['map'])
                ## IY : rqt sec_friction override (>0 overrides; no write-back)
                for i, sec in enumerate(friction_sectors):
                    if i >= 5:
                        break
                    rqt_fric = run_opts.get(f'sec_friction{i}', 0.0)
                    if rqt_fric > 0.0:
                        old_fric = sec.get('friction', 0.0)
                        sec['friction'] = rqt_fric
                        rospy.loginfo(
                            f"[GGTuner] sec{i} friction override: "
                            f"{old_fric:.3f} -> {rqt_fric:.3f} (rqt)")
                ## IY : end
                if friction_sectors:
                    ok, reason = self._generate_sector_ggvs(
                        vehicle_name, friction_sectors, slot_overrides,
                        full_resolution=run_opts['full_resolution'],
                        run_ggv=run_opts['run_ggv'])
                    if not ok:
                        rospy.logerr(
                            f"[GGTuner] Stage 2.5 FAILED: {reason} "
                            f"→ aborting apply")
                        self.status_pub.publish(f"FAILED_STAGE25: {reason}")
                        return
                    self.status_pub.publish(
                        f"SECTOR_GGV_DONE: {vehicle_name}")
                else:
                    rospy.loginfo(
                        "[GGTuner] No sectors → single-GGV mode")
                ## IY : end

                # ---- Stage 3: raceline (optional) ----
                if run_opts['regen_raceline']:
                    ok = self._run_raceline(
                        vehicle_name, run_opts['map'],
                        safety_distance=run_opts['safety_distance'])
                    if not ok:
                        rospy.logerr(f"[GGTuner] raceline failed")
                        return
                else:
                    rospy.loginfo(f"[GGTuner] raceline regen SKIP")

                if run_opts['run_fbga']:
                    force_restart = bool(run_opts['regen_raceline'])
                    ok = self._run_fbga_planner(
                        vehicle_name,
                        enable_mu=run_opts['enable_mu'],
                        force_restart=force_restart)
                    if not ok:
                        return
                    self.status_pub.publish(f"DONE_ALL: {vehicle_name}")
                else:
                    self._kill_fbga_planner()
                    rospy.loginfo(f"[GGTuner] fbga killed (run_fbga=False)")
                    self.status_pub.publish(f"DONE_FBGA_OFF: {vehicle_name}")
                ## IY : end

                rospy.loginfo(f"[GGTuner] ===== Pipeline done: {vehicle_name} =====")

            except Exception as e:
                rospy.logerr(f"[GGTuner] Pipeline exception: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
                self.status_pub.publish(f"EXCEPTION: {str(e)[:100]}")
    ## IY : end
                
    def reconfigure_cb(self, config, level):

        if getattr(config, 'save_now', False):
            if self.pipeline_thread is not None and self.pipeline_thread.is_alive():
                rospy.logwarn(
                    "[GGTuner] SAVE ignored: pipeline running "
                    " ")
            else:
                try:
                    self._snapshot_latest_to_version()
                except Exception as e:
                    rospy.logerr(f"[GGTuner] SAVE exception: {e}")
                    self.status_pub.publish(f"SAVE_EXCEPTION: {str(e)[:80]}")
            config.save_now = False
            if not config.apply:
                return config
        ## IY : end

        if not config.apply:
            return config

        ## IY : refuse concurrent runs
        if self.pipeline_thread is not None and self.pipeline_thread.is_alive():
            rospy.logwarn("[GGTuner] Pipeline already running — ignoring apply")
            config.apply = False
            return config
        ## IY : end

        # collect ALL tuning parameters
        tuning = {k: config[k] for k in self.ALL_TUNING_KEYS}

        run_opts = {
            'run_ggv':          bool(config.run_ggv),
            'full_resolution':  bool(config.full_resolution),
            'regen_raceline':   bool(config.regen_raceline),
            'map':              str(config.map),
            'run_fbga':         bool(config.run_fbga),
            'enable_mu':        bool(config.enable_mu),
            'safety_distance':  float(config.safety_distance),
            ## IY : Y1 mode — per-sector GGV slots (max 5)
            'ggv_sector0':      str(getattr(config, 'ggv_sector0', '')).strip(),
            'ggv_sector1':      str(getattr(config, 'ggv_sector1', '')).strip(),
            'ggv_sector2':      str(getattr(config, 'ggv_sector2', '')).strip(),
            'ggv_sector3':      str(getattr(config, 'ggv_sector3', '')).strip(),
            'ggv_sector4':      str(getattr(config, 'ggv_sector4', '')).strip(),
            ## IY : per-sector friction override (0=use yaml, >0=override)
            'sec_friction0':    float(getattr(config, 'sec_friction0', 0.0)),
            'sec_friction1':    float(getattr(config, 'sec_friction1', 0.0)),
            'sec_friction2':    float(getattr(config, 'sec_friction2', 0.0)),
            'sec_friction3':    float(getattr(config, 'sec_friction3', 0.0)),
            'sec_friction4':    float(getattr(config, 'sec_friction4', 0.0)),
            ## IY : end
        }
        ## IY : end

        ## IY : validate map
        if run_opts['regen_raceline'] and run_opts['map'] not in self.available_maps:
            rospy.logerr(f"[GGTuner] Invalid map '{run_opts['map']}'. "
                         f"Available: {self.available_maps}")
            self.status_pub.publish(f"FAILED_RACELINE: invalid map")
            config.apply = False
            return config
        ## IY : end

        ## IY : background thread
        self.pipeline_thread = threading.Thread(
            target=self._run_full_pipeline,
            args=(tuning, run_opts),
            daemon=True)
        self.pipeline_thread.start()
        rospy.loginfo("[GGTuner] Pipeline spawned in background thread")
        ## IY : end

        config.apply = False
        return config

    def _shutdown_cleanup(self):
        if self.fbga_proc is not None and self.fbga_proc.poll() is None:
            rospy.loginfo("[GGTuner] Terminating FBGA subprocess...")
            try:
                self.fbga_proc.terminate()
                self.fbga_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.fbga_proc.kill()


if __name__ == '__main__':
    rospy.init_node("gg_tuner")
    node = GGTunerNode()
    rospy.spin()
