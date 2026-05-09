import subprocess, json, re
out = subprocess.check_output(
    'source /opt/ros/noetic/setup.bash && source $HOME/catkin_ws/devel/setup.bash && '
    'timeout 3 rostopic echo -n 1 /mpc_auto/debug/tick_json',
    shell=True, executable='/bin/bash').decode()
# rostopic echo prints YAML-ish: data: "..."
m = re.search(r'data:\s*"(.*?)"\n---', out, re.DOTALL)
if not m:
    m = re.search(r'data:\s*"(.*)"', out, re.DOTALL)
s = m.group(1)
# unescape: \\, \", \n yaml folding
s = s.replace('\\"', '"').replace('\\n', '\n').replace('\\\\', '\\')
# rostopic echo wraps long strings with continuation lines that have leading spaces
s = re.sub(r'\n\s+', '', s)
try:
    d = json.loads(s)
    print('keys (last 12):', list(d.keys())[-12:])
    print('plan:', d.get('plan'))
    print('obs_in_horizon:', d.get('obs_in_horizon'))
    pv = d.get('pred_variance', {})
    print('max_vd_var:', pv.get('max_vd_var'))
except Exception as e:
    print('err:', e)
    print('first 400:', s[:400])
