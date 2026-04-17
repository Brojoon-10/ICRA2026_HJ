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
## IY : gg_tuner_3d_node clones gg_tuner_node but Stage 4 drives
#       3d_optimized_vel_planner.py (NLP velocity re-optimization) instead
#       of FBGA. Uses GGTuner3DConfig (independent cfg) so the two tuners
#       coexist on rqt_reconfigure without parameter-namespace collisions.
from stack_master.cfg import GGTuner3DConfig as GGTunerConfig
## IY : end
## IY : Trigger service for /velopt/reload (Stage 4 hot-reload)
from std_srvs.srv import Trigger
## IY : end


class GGTunerNode:

    ### IY(0410) : 튜닝 대상 파라미터 키 분류
    TIRE_KEYS = ['lambda_mu_x', 'lambda_mu_y', 'p_Dx_2', 'p_Dy_2']
    ## IY : VEHICLE_KEYS 에 cap 포함, POST_KEYS + RACELINE_KEYS 신규
    VEHICLE_KEYS = ['P_max', 'v_max', 'epsilon',
                    'P_brake_max', 'ax_max_cap', 'ax_min_cap', 'ay_max_cap']
    CAP_KEYS = ['P_brake_max', 'ax_max_cap', 'ax_min_cap', 'ay_max_cap']
    POST_KEYS = ['gg_exp_scale', 'ax_max_scale', 'ax_min_scale', 'ay_scale']
    RACELINE_KEYS = ['V_min', 'safety_distance', 'w_T', 'w_jx', 'w_jy', 'w_dOmega_z']
    ALL_TUNING_KEYS = TIRE_KEYS + VEHICLE_KEYS + POST_KEYS + RACELINE_KEYS
    ## IY : end

    def __init__(self):
        rospy.loginfo("[GGTuner] Initializing...")

        ### IY(0410) : 경로 설정
        script_dir = os.path.dirname(os.path.abspath(__file__))
        race_stack_root = os.path.dirname(os.path.dirname(script_dir))
        self.race_stack_root = race_stack_root

        self.data_path = os.path.join(
            race_stack_root, 'planner', '3d_gb_optimizer', 'global_line', 'data')
        self.maps_dir = os.path.join(race_stack_root, 'stack_master', 'maps')

        ## IY : fast_ggv_gen paths (primary GGV engine, legacy removed)
        self.fast_ggv_dir = os.path.join(
            race_stack_root, 'planner', '3d_gb_optimizer', 'fast_ggv_gen')
        self.fast_ggv_script = os.path.join(self.fast_ggv_dir, 'run_on_container.sh')
        self.fast_ggv_output_dir = os.path.join(self.fast_ggv_dir, 'output')
        ## IY : end

        ### IY(0410) : base vehicle 이름
        self.base_vehicle = rospy.get_param("~base_vehicle", "rc_car_10th")

        ## IY : base params 로드 — fallback to _backup.yml if main is broken
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
        ## IY : end

        ## IY : script existence warning
        if not os.path.exists(self.fast_ggv_script):
            rospy.logwarn(f"[GGTuner] fast_ggv script missing: {self.fast_ggv_script}")
        ## IY : end

        ### IY(0410) : 상태 토픽 (latched)
        self.status_pub = rospy.Publisher(
            '/gg_compute_status', String, queue_size=5, latch=True)
        self.status_pub.publish(f"READY: {self.base_vehicle}")

        ## IY : background pipeline state
        self.pipeline_thread = None
        self.pipeline_lock = threading.Lock()
        self.velopt_proc = None
        ## IY : end

        ## IY : cleanup hook
        rospy.on_shutdown(self._shutdown_cleanup)
        ## IY : end

        ### IY(0410) : dynamic_reconfigure 서버
        self.srv = Server(GGTunerConfig, self.reconfigure_cb)
        rospy.loginfo("[GGTuner] Ready. Use rqt_reconfigure → /gg_tuner")

    # ==================================================================
    # Cache / versioning helpers
    # ==================================================================
    def _round_tuning(self, tuning_dict):
        """부동소수점 오차 방지용 소수점 4자리 반올림"""
        return {k: round(v, 4) for k, v in sorted(tuning_dict.items())}

    def _find_cached(self, tuning_dict):
        """기존 v* 디렉토리에서 동일 튜닝 조합 찾기. symlink 스킵."""
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
        """v<N> 중 최대 N+1 반환"""
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

    # ==================================================================
    ## IY : unified merge — ALL keys in one yml
    # ==================================================================
    def _merge_all_params(self, tuning_dict):
        """
        base params + 전체 tuning override → 하나의 통합 dict 생성.
        - TIRE/VEHICLE keys → vehicle_params/tire_params 섹션에 merge
        - CAP keys 는 0.0 → None 변환
        - POST/RACELINE keys → top-level 에 추가
        """
        merged = copy.deepcopy(self.base_params)
        # NLP: tire
        for key in self.TIRE_KEYS:
            if key in tuning_dict:
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
        """통합 yml 저장 + latest 에 실제 파일 복사 (symlink 아님)"""
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
        """캐시 메타 저장"""
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

    ## IY : latest 전용 헬퍼 (save_now 기반 스냅샷 워크플로우)
    def _write_latest_params_yml(self, merged_params):
        """params_<base>_latest.yml 에 직접 덮어쓰기 (중간 v<N>.yml 안 만듦)."""
        latest_yml = os.path.join(
            self.data_path, 'vehicle_params',
            f'params_{self.base_vehicle}_latest.yml')
        with open(latest_yml, 'w') as f:
            yaml.dump(merged_params, f, default_flow_style=False,
                      allow_unicode=True)
        rospy.loginfo(f"[GGTuner] latest yml written: {latest_yml}")
        return latest_yml

    def _restore_to_latest(self, cached_name):
        """저장된 v<N> 스냅샷을 latest 로 복사 (cache hit 시).
        gg_diagrams/<cached_name>/ → gg_diagrams/<base>_latest/,
        params_<cached_name>.yml → params_<base>_latest.yml.
        """
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
        # gg_diagrams: 기존 latest 지우고 재복사
        if os.path.islink(dst_gg):
            os.unlink(dst_gg)
        elif os.path.exists(dst_gg):
            shutil.rmtree(dst_gg)
        shutil.copytree(src_gg, dst_gg)
        # yml: 있으면 복사
        if os.path.exists(src_yml):
            shutil.copy2(src_yml, dst_yml)
        rospy.loginfo(f"[GGTuner] restored {cached_name} → latest")
        return True

    def _snapshot_latest_to_version(self, tuning_dict=None):
        """현재 latest 를 v<N+1> 로 영구 저장 (save_now 체크 시 호출).
        tuning_dict 가 None 이면 latest 의 params_used.json 에서 읽어서 사용.
        """
        latest_name = f'{self.base_vehicle}_latest'
        latest_gg = os.path.join(self.data_path, 'gg_diagrams', latest_name)
        latest_yml = os.path.join(self.data_path, 'vehicle_params',
                                  f'params_{latest_name}.yml')
        if not os.path.exists(latest_gg):
            rospy.logerr(f"[GGTuner] SAVE failed: latest gg_diagrams missing: {latest_gg}")
            self.status_pub.publish("SAVE_FAILED: no latest gg")
            return False
        if not os.path.exists(latest_yml):
            rospy.logerr(f"[GGTuner] SAVE failed: latest yml missing: {latest_yml}")
            self.status_pub.publish("SAVE_FAILED: no latest yml")
            return False

        ver = self._next_version()
        snapshot_name = f'{self.base_vehicle}_v{ver}'
        rospy.loginfo(f"[GGTuner] ===== SAVE snapshot: {snapshot_name} =====")
        self.status_pub.publish(f"SAVING: {snapshot_name}")

        # 1) gg_diagrams 복사 (심볼릭이면 실제 타겟까지 따라감)
        dst_gg = os.path.join(self.data_path, 'gg_diagrams', snapshot_name)
        real_src = os.path.realpath(latest_gg) if os.path.islink(latest_gg) else latest_gg
        shutil.copytree(real_src, dst_gg)

        # 2) yml 복사
        dst_yml = os.path.join(self.data_path, 'vehicle_params',
                               f'params_{snapshot_name}.yml')
        shutil.copy2(latest_yml, dst_yml)

        # 3) meta: latest 의 params_used.json 에서 tuning 을 가져와 vehicle_name 갱신,
        #    없으면 인자로 받은 tuning_dict 로 새로 작성.
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
        #   latest_f{NNN}/ → v{ver}_f{NNN}/  (+ corresponding yml)
        gg_parent = os.path.join(self.data_path, 'gg_diagrams')
        fric_prefix = f'{latest_name}_f'
        for name in os.listdir(gg_parent):
            if not name.startswith(fric_prefix):
                continue
            fric_suffix = name[len(latest_name):]          # e.g. "_f045"
            src_fric_gg = os.path.join(gg_parent, name)
            dst_fric_gg = os.path.join(gg_parent, f'{snapshot_name}{fric_suffix}')
            if os.path.isdir(src_fric_gg) and not os.path.islink(src_fric_gg):
                shutil.copytree(src_fric_gg, dst_fric_gg)
                # copy friction yml if exists
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

    # ==================================================================
    # Helpers
    # ==================================================================
    def _scan_maps_dir(self):
        if not os.path.exists(self.maps_dir):
            return []
        return sorted([n for n in os.listdir(self.maps_dir)
                       if os.path.isdir(os.path.join(self.maps_dir, n))])

    ### HJ : npy 묶음 → gg.bin 직렬화 (FBGA C++ runner 입력 포맷)
    #        FBGA 노드의 _generate_gg_bin과 동일 포맷. GGV 재계산 시
    #        gg_diagrams/<vehicle>/velocity_frame/gg.bin 을 함께 갱신해
    #        FBGA reload 가 cold start 폴백 없이 바로 새 GGV 를 읽도록 함.
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
        """fast_ggv output → gg_diagrams 복사"""
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
        ### HJ : npy 갱신 직후 gg.bin 도 같이 재생성 (vehicle_frame/velocity_frame 양쪽)
        for frame in ('velocity_frame', 'vehicle_frame'):
            frame_dir = os.path.join(dst, frame)
            if os.path.isdir(frame_dir):
                self._generate_gg_bin(frame_dir)
        return True

    ## IY : symlinks for DIRECTORIES only (params latest is a real file copy)
    def _update_dir_symlinks(self, vehicle_name):
        """gg_diagrams + fast_ggv output 의 _latest 심볼릭만 갱신"""
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
        """subprocess stdout 을 rospy.loginfo 로 스트리밍"""
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

    # ==================================================================
    # Stage runners
    # ==================================================================
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

    ## IY(0416) : friction sector별 GGV 병렬 생성
    #   friction_scaling.yaml의 friction 값 = p_Dx_1 = p_Dy_1 (절대값).
    #   각 unique friction에 대해 base params에서 p_Dx_1/p_Dy_1만 교체한 뒤
    #   fast_ggv_gen을 병렬로 실행. velopt/FBGA가 sector별 GGV를 참조.
    def _read_friction_sectors(self, map_name):
        """Read friction sectors — rosparam first (live rqt values), yaml fallback."""
        # --- (original: yaml-only read, kept for reference) ---
        # yaml_path = os.path.join(self.maps_dir, map_name, 'friction_scaling.yaml')
        # with open(yaml_path) as f: data = yaml.safe_load(f)
        # --- (end) ---
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
        # 2) fallback: read yaml file directly
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

    def _generate_friction_ggvs(self, vehicle_name, sectors, full_resolution=False):
        """friction sector별 GGV 병렬 생성.

        각 unique friction에 대해:
        1) base params yml 복사 → p_Dx_1, p_Dy_1을 friction 값으로 교체
        2) fast_ggv_gen 실행 → {vehicle_name}_f{NNN}/ 에 저장
        3) gg_diagrams로 복사

        base p_Dx_1과 동일한 friction은 skip (이미 생성됨).
        """
        if not sectors:
            return True

        # base params에서 p_Dx_1 읽기
        base_yml = os.path.join(
            self.data_path, 'vehicle_params', f'params_{vehicle_name}.yml')
        if not os.path.exists(base_yml):
            rospy.logwarn(f"[GGTuner] base yml not found: {base_yml}")
            return False
        with open(base_yml) as f:
            base_params = yaml.safe_load(f)
        base_p_Dx_1 = base_params.get('tire_params', {}).get('p_Dx_1', 0.56)

        # unique friction 추출 (base와 동일한 건 제외)
        unique_frictions = sorted(set(s['friction'] for s in sectors))
        to_generate = [f for f in unique_frictions
                       if abs(f - base_p_Dx_1) > 1e-4]

        if not to_generate:
            rospy.loginfo("[GGTuner] All friction sectors match base p_Dx_1 "
                          "→ no extra GGVs needed")
            return True

        rospy.loginfo(f"[GGTuner] Generating friction GGVs: {to_generate} "
                      f"(base p_Dx_1={base_p_Dx_1:.3f})")
        self.status_pub.publish(
            f"FRICTION_GGV: {len(to_generate)} variants")

        # 각 friction에 대해 params yml 생성 + fast_ggv_gen 실행
        threads = []
        results = {}
        for fric in to_generate:
            fric_int = int(round(fric * 100))
            fric_vehicle = f"{vehicle_name}_f{fric_int:03d}"

            # params yml 복사 + p_Dx_1, p_Dy_1 교체
            fric_params = copy.deepcopy(base_params)
            fric_params['tire_params']['p_Dx_1'] = fric
            fric_params['tire_params']['p_Dy_1'] = fric
            fric_yml = os.path.join(
                self.data_path, 'vehicle_params',
                f'params_{fric_vehicle}.yml')
            with open(fric_yml, 'w') as f:
                yaml.dump(fric_params, f, default_flow_style=False,
                          allow_unicode=True)
            rospy.loginfo(f"[GGTuner] friction yml: {fric_yml} "
                          f"(p_Dx_1={fric}, p_Dy_1={fric})")

            # 병렬 실행
            def _run_one(vname, fval, res_dict):
                ok = self._run_fast_ggv(vname, full_resolution)
                if ok:
                    ok = self._copy_to_gg_diagrams(vname)
                res_dict[fval] = ok

            t = threading.Thread(target=_run_one,
                                 args=(fric_vehicle, fric, results))
            t.start()
            threads.append(t)

        for t in threads:
            t.join()

        failed = [f for f, ok in results.items() if not ok]
        if failed:
            rospy.logerr(f"[GGTuner] Friction GGV failed for: {failed}")
            return False

        rospy.loginfo(f"[GGTuner] All friction GGVs generated: {to_generate}")
        return True
    ## IY(0416) : end

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

    ## IY : _run_fbga_planner 재작성 — hot-reload 우선, 실패 시 cold start.
    #       원본 로직은 항상 cold restart 였음 (rosnode kill + sleep + Popen).
    # --- (기존 _run_fbga_planner 원본, 보존용 주석) ---
    # def _run_fbga_planner(self, vehicle_name, enable_mu=True):
    #     rospy.loginfo(f"[GGTuner] [fbga] restarting: {vehicle_name}, enable_mu={enable_mu}")
    #     self.status_pub.publish(f"FBGA_STARTED: {vehicle_name}")
    #     try:
    #         subprocess.run(['rosnode', 'kill', '/fbga_planner'],
    #                        capture_output=True, timeout=5, check=False)
    #         time.sleep(1)
    #     except (subprocess.TimeoutExpired, FileNotFoundError):
    #         pass
    #     if self.velopt_proc is not None and self.velopt_proc.poll() is None:
    #         try:
    #             self.velopt_proc.terminate()
    #             self.velopt_proc.wait(timeout=3)
    #         except subprocess.TimeoutExpired:
    #             self.velopt_proc.kill()
    #     self.velopt_proc = None
    #     gg_bin = os.path.join(
    #         self.data_path, 'gg_diagrams', vehicle_name,
    #         'velocity_frame', 'gg.bin')
    #     params_yml = os.path.join(
    #         self.data_path, 'vehicle_params',
    #         'params_' + vehicle_name + '.yml')
    #     if not os.path.exists(params_yml):
    #         rospy.logerr(f"[GGTuner] params yml missing: {params_yml}")
    #         self.status_pub.publish(f"FAILED_FBGA: {vehicle_name}")
    #         return False
    #     cmd = [
    #         'rosrun', 'stack_master', 'fbga_velocity_planner.py',
    #         '__name:=fbga_planner',
    #         f'_gg_bin:={gg_bin}',
    #         f'_params_yml:={params_yml}',
    #         f'_enable_mu:={str(enable_mu).lower()}',
    #     ]
    #     rospy.loginfo(f"[GGTuner] [fbga] launching: {' '.join(cmd)}")
    #     try:
    #         self.velopt_proc = subprocess.Popen(cmd)
    #         return True
    #     except OSError as e:
    #         rospy.logerr(f"[GGTuner] Failed to start FBGA: {e}")
    #         self.status_pub.publish(f"FAILED_FBGA: {vehicle_name}")
    #         return False
    # --- (원본 끝) ---
    ## IY : Stage 4 replacement — FBGA → 3d_optimized_vel_planner (velopt).
    #       Pattern mirrors FBGA exactly: hot-reload via /velopt/reload first,
    #       cold-start fallback with rosrun. Original FBGA methods preserved
    #       above as comments for reference.
    # --- (original _run_fbga_planner signature was:
    #      _run_fbga_planner(self, vehicle_name, enable_mu=True, force_restart=False)
    #      — enable_mu is FBGA-specific and unused here. velopt_opts dict carries
    #      VelOpt NLP knobs (velopt_V_min, velopt_gg_margin, velopt_step_size).) ---
    def _run_velopt_planner(self, vehicle_name, map_name, velopt_opts,
                            force_restart=False):
        """VelOpt 업데이트. 살아있으면 /velopt/reload, 없으면 cold start.

        force_restart=True: reload 건너뛰고 cold start. raceline 이 regen 되어
        새 /global_waypoints 가 publish된 경우 템플릿 재subscribe 포함해서
        프로세스 자체를 갈아끼우는 게 안전.

        Args:
            vehicle_name : GGV+raceline variant name (e.g. rc_car_10th_latest)
            map_name     : map folder name (stack_master/maps/<map>/)
            velopt_opts  : dict with keys V_min, gg_margin, step_size, w_T, w_jx
        """
        rospy.loginfo(
            f"[GGTuner] [velopt] update: vehicle={vehicle_name}, map={map_name}, "
            f"opts={velopt_opts}, force_restart={force_restart}")
        self.status_pub.publish(f"VELOPT_STARTED: {vehicle_name}")

        # Precondition: params yml + raceline csv exist (velopt reads them).
        params_yml = os.path.join(
            self.data_path, 'vehicle_params',
            'params_' + vehicle_name + '.yml')
        if not os.path.exists(params_yml):
            rospy.logerr(f"[GGTuner] params yml missing: {params_yml}")
            self.status_pub.publish(f"FAILED_VELOPT: {vehicle_name}")
            return False
        raceline_csv = os.path.join(
            self.maps_dir, map_name,
            f'{map_name}_3d_{vehicle_name}_timeoptimal.csv')
        if not os.path.exists(raceline_csv):
            rospy.logerr(f"[GGTuner] raceline csv missing: {raceline_csv}")
            self.status_pub.publish(f"FAILED_VELOPT: {vehicle_name}")
            return False

        # Parameter server refresh — used by both reload and cold start paths.
        # velopt._load_and_solve() reads these as ~map / ~racecar / ~V_min etc.
        rospy.set_param('/vel_opt_3d/map',           map_name)
        rospy.set_param('/vel_opt_3d/racecar',       vehicle_name)
        rospy.set_param('/vel_opt_3d/V_min',         float(velopt_opts['V_min']))
        rospy.set_param('/vel_opt_3d/gg_margin',     float(velopt_opts['gg_margin']))
        rospy.set_param('/vel_opt_3d/step_size_opt', float(velopt_opts['step_size']))
        ## IY : NLP cost weights (time vs jerk smoothness)
        rospy.set_param('/vel_opt_3d/w_T',           float(velopt_opts['w_T']))
        rospy.set_param('/vel_opt_3d/w_jx',          float(velopt_opts['w_jx']))
        ## IY : end

        # --- hot-reload first ---
        if not force_restart:
            try:
                rospy.wait_for_service('/velopt/reload', timeout=0.5)
                reload_srv = rospy.ServiceProxy('/velopt/reload', Trigger)
                resp = reload_srv()
                if resp.success:
                    rospy.loginfo(
                        f"[GGTuner] [velopt] hot-reloaded: {resp.message}")
                    self.status_pub.publish(f"VELOPT_DONE: {vehicle_name}")
                    return True
                else:
                    rospy.logwarn(
                        f"[GGTuner] [velopt] reload returned failure: "
                        f"{resp.message} → cold start")
            except (rospy.ROSException, rospy.ServiceException) as e:
                rospy.loginfo(
                    f"[GGTuner] [velopt] reload unavailable ({e}) → cold start")

        # --- cold start: kill any surviving vel_opt_3d and respawn ---
        try:
            subprocess.run(['rosnode', 'kill', '/vel_opt_3d'],
                           capture_output=True, timeout=5, check=False)
            time.sleep(0.3)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        if self.velopt_proc is not None and self.velopt_proc.poll() is None:
            try:
                self.velopt_proc.terminate()
                self.velopt_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.velopt_proc.kill()
        self.velopt_proc = None

        cmd = [
            'rosrun', 'stack_master', '3d_optimized_vel_planner.py',
            '__name:=vel_opt_3d',
            f'_map:={map_name}',
            f'_racecar:={vehicle_name}',
            f'_V_min:={float(velopt_opts["V_min"])}',
            f'_gg_margin:={float(velopt_opts["gg_margin"])}',
            f'_step_size_opt:={float(velopt_opts["step_size"])}',
            ## IY : NLP cost weights
            f'_w_T:={float(velopt_opts["w_T"])}',
            f'_w_jx:={float(velopt_opts["w_jx"])}',
            ## IY : end
        ]
        rospy.loginfo(f"[GGTuner] [velopt] cold start: {' '.join(cmd)}")
        try:
            self.velopt_proc = subprocess.Popen(cmd)
            return True
        except OSError as e:
            rospy.logerr(f"[GGTuner] Failed to start velopt: {e}")
            self.status_pub.publish(f"FAILED_VELOPT: {vehicle_name}")
            return False

    def _kill_velopt_planner(self):
        """Stage 4 노드(/vel_opt_3d) 완전 종료 (run_velopt=False 체크 해제 시)."""
        rospy.loginfo("[GGTuner] [velopt] kill requested (checkbox off)")
        try:
            subprocess.run(['rosnode', 'kill', '/vel_opt_3d'],
                           capture_output=True, timeout=5, check=False)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        if self.velopt_proc is not None and self.velopt_proc.poll() is None:
            try:
                self.velopt_proc.terminate()
                self.velopt_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.velopt_proc.kill()
        self.velopt_proc = None
        rospy.loginfo("[GGTuner] [velopt] killed")
    ## IY : end

    # ==================================================================
    ## IY : full pipeline (background thread)
    # ==================================================================
    def _run_full_pipeline(self, tuning, run_opts):
        with self.pipeline_lock:
            try:
                rospy.loginfo(f"[GGTuner] ===== Pipeline start =====")
                rospy.loginfo(f"[GGTuner] tuning: {tuning}")
                rospy.loginfo(f"[GGTuner] options: {run_opts}")
                self.status_pub.publish(f"STARTED: {self.base_vehicle}")

                ## IY : 파이프라인은 항상 latest 에 덮어쓰기 (중간 v<N> 파일 생성 X).
                #       v<N> 저장은 save_now 체크박스로만 수동 트리거됨 → _snapshot_latest_to_version.
                # --- (기존 Stage 1/2 원본, 보존용 주석) ---
                # if not run_opts['run_ggv']:
                #     latest_link = os.path.join(
                #         self.data_path, 'gg_diagrams',
                #         f'{self.base_vehicle}_latest')
                #     if os.path.exists(latest_link):
                #         vehicle_name = os.path.basename(os.readlink(latest_link)) \
                #             if os.path.islink(latest_link) else f'{self.base_vehicle}_latest'
                #     else:
                #         vehicle_name = self.base_vehicle
                #     self.status_pub.publish(f"GGV_SKIP: {vehicle_name}")
                # else:
                #     cached_name = self._find_cached(tuning)
                #     if cached_name is not None:
                #         vehicle_name = cached_name
                #         self.status_pub.publish(f"CACHED: {vehicle_name}")
                #         self._save_params_yml(vehicle_name,
                #                               self._merge_all_params(tuning))
                #         self._update_dir_symlinks(vehicle_name)
                #     else:
                #         ver = self._next_version()
                #         vehicle_name = f"{self.base_vehicle}_v{ver}"
                #         merged = self._merge_all_params(tuning)
                #         self._save_params_yml(vehicle_name, merged)
                #         ok = self._run_fast_ggv(vehicle_name,
                #                                 full_resolution=run_opts['full_resolution'])
                #         if not ok: ... return
                #         if not self._copy_to_gg_diagrams(vehicle_name): ... return
                #         self._save_meta(vehicle_name, tuning)
                #         self._update_dir_symlinks(vehicle_name)
                #         self.status_pub.publish(f"GGV_DONE: {vehicle_name}")
                # --- (원본 끝) ---
                vehicle_name = f"{self.base_vehicle}_latest"

                ### HJ : 사용자 지정 GGV 이름 처리.
                #   비어있으면 latest 그대로 사용 (기본 동작 유지).
                #   이름이 주어지면 gg_diagrams/<name>/ 존재 확인 후
                #     - 존재: 해당 스냅샷을 latest 로 복원 (params yml 포함)
                #             → Stage 2 (GGV 생성) SKIP 하고 Stage 3/4 로 진행.
                #     - 부재: warning 출력 + 파이프라인은 기존 흐름대로 계속.
                user_ggv = run_opts.get('ggv_name', '')
                ggv_override_applied = False
                if user_ggv:
                    src_gg = os.path.join(
                        self.data_path, 'gg_diagrams', user_ggv)
                    if os.path.exists(src_gg):
                        rospy.loginfo(
                            f"[GGTuner] ggv_name='{user_ggv}' → restore to latest")
                        if self._restore_to_latest(user_ggv):
                            self.status_pub.publish(f"GGV_USER: {user_ggv}")
                            ggv_override_applied = True
                        else:
                            rospy.logwarn(
                                f"[GGTuner] ggv_name='{user_ggv}' restore failed "
                                f"→ 기본 파이프라인 진행")
                    else:
                        rospy.logwarn(
                            f"[GGTuner] ggv_name='{user_ggv}' not found at "
                            f"{src_gg} → 기본 파이프라인 진행 (latest 사용)")

                if ggv_override_applied:
                    rospy.loginfo(
                        f"[GGTuner] GGV override: skip Stage 2 "
                        f"(using restored latest from '{user_ggv}')")
                elif not run_opts['run_ggv']:
                    # GGV skip — 기존 latest 그대로 사용
                    latest_gg = os.path.join(
                        self.data_path, 'gg_diagrams', vehicle_name)
                    if not os.path.exists(latest_gg):
                        rospy.logerr(
                            f"[GGTuner] latest gg_diagrams missing: {latest_gg}")
                        self.status_pub.publish("FAILED: no latest")
                        return
                    rospy.loginfo(
                        f"[GGTuner] GGV skip (run_ggv=False), using latest")
                    self.status_pub.publish(f"GGV_SKIP: {vehicle_name}")
                else:
                    # ---- Cache check: 저장된 v<N> 중 같은 tuning 있으면 latest 로 복사 ----
                    cached_name = self._find_cached(tuning)
                    if cached_name is not None:
                        rospy.loginfo(
                            f"[GGTuner] cache hit: {cached_name} → restore to latest")
                        self.status_pub.publish(f"CACHED: {cached_name}")
                        if not self._restore_to_latest(cached_name):
                            self.status_pub.publish("FAILED: restore")
                            return
                        # params.yml latest 도 현재 tuning 으로 재기록 (merged 기준)
                        self._write_latest_params_yml(self._merge_all_params(tuning))
                    else:
                        # ---- Cache miss: latest 에 직접 계산/저장 ----
                        rospy.loginfo(
                            f"[GGTuner] cache miss → compute fresh into latest")
                        merged = self._merge_all_params(tuning)
                        # 1) latest yml 먼저 기록 (fast_ggv 가 이걸 읽음)
                        self._write_latest_params_yml(merged)
                        # 2) fast_ggv 실행 (output/<latest_name>/ 에 생성)
                        ok = self._run_fast_ggv(
                            vehicle_name,
                            full_resolution=run_opts['full_resolution'])
                        if not ok:
                            self.status_pub.publish(f"FAILED_GGV: {vehicle_name}")
                            return
                        # 3) fast_ggv output → gg_diagrams/latest/ 복사
                        if not self._copy_to_gg_diagrams(vehicle_name):
                            self.status_pub.publish(f"FAILED_GGV: {vehicle_name}")
                            return
                        # 4) meta 저장 (tuning 기록 — save 시 v<N> 로 복사됨)
                        self._save_meta(vehicle_name, tuning)
                        self.status_pub.publish(f"GGV_DONE: {vehicle_name}")
                ## IY : end

                ## IY(0416) : Stage 2.5 — friction sector별 GGV 병렬 생성
                #   base GGV 생성/복원 후, friction_scaling.yaml의 sector별 friction으로
                #   추가 GGV를 생성. velopt NLP가 per-waypoint로 참조.
                if run_opts['run_ggv'] or ggv_override_applied:
                    friction_sectors = self._read_friction_sectors(run_opts['map'])
                    if friction_sectors:
                        ok = self._generate_friction_ggvs(
                            vehicle_name, friction_sectors,
                            full_resolution=run_opts['full_resolution'])
                        if ok:
                            self.status_pub.publish(
                                f"FRICTION_GGV_DONE: {vehicle_name}")
                        else:
                            rospy.logwarn(
                                "[GGTuner] Friction GGV generation failed "
                                "→ velopt will use single GGV fallback")
                ## IY(0416) : end

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

                ## IY : Stage 4 재작성 — run_fbga=True 는 reload 우선, False 는 kill.
                #       raceline 이 regen 된 경우(force_restart=True) cold start 강제
                #       (reload는 캐시된 옛 wpnts 를 쓰므로 새 raceline 반영 불가).
                # --- (기존 Stage 4 원본, 보존용 주석) ---
                # if run_opts['run_fbga']:
                #     ok = self._run_velopt_planner(vehicle_name,
                #                                 enable_mu=run_opts['enable_mu'])
                #     if not ok:
                #         return
                #     self.status_pub.publish(f"DONE_ALL: {vehicle_name}")
                # else:
                #     rospy.loginfo(f"[GGTuner] fbga restart SKIP")
                #     self.status_pub.publish(f"DONE: {vehicle_name}")
                # --- (원본 끝) ---
                ## IY : Stage 4 invoke — VelOpt (velopt_opts from run_opts).
                if run_opts['run_velopt']:
                    force_restart = bool(run_opts['regen_raceline'])
                    velopt_opts = {
                        'V_min':     run_opts['velopt_V_min'],
                        'gg_margin': run_opts['velopt_gg_margin'],
                        'step_size': run_opts['velopt_step_size'],
                        ## IY : NLP cost weights
                        'w_T':       run_opts['velopt_w_T'],
                        'w_jx':      run_opts['velopt_w_jx'],
                        ## IY : end
                    }
                    ok = self._run_velopt_planner(
                        vehicle_name,
                        map_name=run_opts['map'],
                        velopt_opts=velopt_opts,
                        force_restart=force_restart)
                    if not ok:
                        return
                    self.status_pub.publish(f"DONE_ALL: {vehicle_name}")
                else:
                    ### HJ : _kill_fbga_planner → _kill_velopt_planner (3D 노드 전용)
                    # run_velopt=False → velopt 노드 완전 종료
                    self._kill_velopt_planner()
                    rospy.loginfo(f"[GGTuner] velopt killed (run_velopt=False)")
                    self.status_pub.publish(f"DONE_VELOPT_OFF: {vehicle_name}")
                ## IY : end

                rospy.loginfo(f"[GGTuner] ===== Pipeline done: {vehicle_name} =====")

            except Exception as e:
                rospy.logerr(f"[GGTuner] Pipeline exception: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
                self.status_pub.publish(f"EXCEPTION: {str(e)[:100]}")
    ## IY : end

    # ==================================================================
    # dynamic_reconfigure callback
    # ==================================================================
    def reconfigure_cb(self, config, level):
        ## IY : save_now 는 apply 와 독립 trigger — 현재 latest 를 v<N+1> 로 복사.
        #       apply 체크 여부와 무관하게 save_now 가 True 면 즉시 스냅샷 후 return.
        #       파이프라인이 돌고있으면 무시 (중간 상태 저장 방지).
        if getattr(config, 'save_now', False):
            if self.pipeline_thread is not None and self.pipeline_thread.is_alive():
                rospy.logwarn(
                    "[GGTuner] SAVE ignored: pipeline running "
                    "(apply 중간에 저장하면 불완전한 상태가 될 수 있음)")
            else:
                try:
                    self._snapshot_latest_to_version()
                except Exception as e:
                    rospy.logerr(f"[GGTuner] SAVE exception: {e}")
                    self.status_pub.publish(f"SAVE_EXCEPTION: {str(e)[:80]}")
            config.save_now = False
            # apply 가 같이 체크돼 있지 않으면 여기서 끝
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

        ## IY : collect pipeline options — GGTuner3D swaps FBGA keys (run_fbga,
        #       enable_mu) for VelOpt keys (run_velopt, velopt_V_min,
        #       velopt_gg_margin, velopt_step_size).
        run_opts = {
            'run_ggv':           bool(config.run_ggv),
            'full_resolution':   bool(config.full_resolution),
            'regen_raceline':    bool(config.regen_raceline),
            'map':               str(config.map),
            'run_velopt':        bool(config.run_velopt),
            'velopt_V_min':      float(config.velopt_V_min),
            'velopt_gg_margin':  float(config.velopt_gg_margin),
            'velopt_step_size':  float(config.velopt_step_size),
            ## IY : VelOpt NLP cost weights (separate from Raceline.w_T/w_jx)
            'velopt_w_T':        float(config.velopt_w_T),
            'velopt_w_jx':       float(config.velopt_w_jx),
            ## IY : end
            'safety_distance':   float(config.safety_distance),
            'ggv_name':          str(getattr(config, 'ggv_name', '')).strip(),
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

    # ==================================================================
    def _shutdown_cleanup(self):
        if self.velopt_proc is not None and self.velopt_proc.poll() is None:
            rospy.loginfo("[GGTuner] Terminating FBGA subprocess...")
            try:
                self.velopt_proc.terminate()
                self.velopt_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.velopt_proc.kill()


if __name__ == '__main__':
    ## IY : distinct node name so gg_tuner and gg_tuner_3d can coexist
    rospy.init_node("gg_tuner_3d")
    ## IY : end
    node = GGTunerNode()
    rospy.spin()
