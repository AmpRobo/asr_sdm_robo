#!/bin/bash
# Sweep sparse_align hyperparams on MH01 and dump the last
# [SPARSE_STATS] / [TRACKER_STATS] / [KLT_STATS] from each log into a
# single CSV. We mutate install/vins.yaml (we always restore it at end),
# restart the node via run_one.sh, parse out the steady-state bins.
set -eu

WS=/home/lxy/asr_sdm_robo
SRC_YAML=$WS/src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/vins_estimator/config/vins.yaml
INSTALL_YAML=$WS/install/vins_estimator/share/vins_estimator/config/vins.yaml
SEQ=${SEQ:-MH01}
BAG=datasheet/MH_${SEQ#MH}_easy_ros2
OUT=$WS/output/$SEQ/sweep_$(date +%H%M%S)
mkdir -p "$OUT"
RESULT=$OUT/results.csv
echo "tag,sparse_time_ms,klt_time_ms,mean_total_ms,sparse_pct,klt_pct,sparse_prior_pct,chi2,succ,nmeas,win,lvl" > "$RESULT"

restore() {
  cp -f "$SRC_YAML" "$INSTALL_YAML"
  exit 0
}
trap restore EXIT INT TERM

cat > "$OUT/sweep.csv" <<'EOF'
tag,patch,max_level,min_level,max_iter,chi2_thresh,min_features
A,2,1,0,2,50,30
B,2,1,0,1,50,30
C,2,0,0,2,50,30
D,2,1,0,1,30,30
E,2,1,0,1,50,20
EOF

while IFS=',' read -r tag patch ml minl mi chi mf; do
  [[ "$tag" == "tag" ]] && continue
  echo "=== [$tag] patch=$patch levels=[$minl,$ml] iter=$mi chi=$chi mf=$mf ==="
  cat > "$INSTALL_YAML" <<YAML
/**:
  ros__parameters:
    params_file: config/euroc/euroc_config.yaml
    calibration_file: config/euroc/euroc_cam_calibration.yaml
    config_file: ""
    enable_sparse: false
    sparse_align:
      use_sparse_align: 1
      use_td_pre_calib: 0
      sparse_align_patch_size: $patch
      sparse_align_max_level: $ml
      sparse_align_min_level: $minl
      sparse_align_max_iter: $mi
      sparse_align_lambda_rot: 0.5
      sparse_align_lambda_trans: 0.0
      sparse_align_chi2_thresh: $chi
      sparse_align_min_features: $mf
      sparse_align_min_iter_for_ok: 1
    namespace: localization/vins
    rviz_config: config/vins_euroc_rviz.rviz
    pose_graph:
      visualization_shift_x: 0
      visualization_shift_y: 0
      skip_cnt: 0
      skip_dis: 0.0
YAML

  bash "$WS/experiments/sparse_compare/scripts/run_one.sh" \
    --mode on --bag "$BAG" --seq "$SEQ" --extra-wait 6 \
    > "$OUT/${tag}.log" 2>&1 || true
  LOG="$OUT/${tag}.log"
  cp "$WS/output/$SEQ/sparse_on/vins_sparse_on.csv" \
     "$OUT/${tag}_traj.csv" 2>/dev/null || true
  # tail bins
  echo "  banner: $(grep 'sparse_align enabled' $LOG | head -1)"
  echo "  TRACKER last 3:"
  grep TRACKER_STATS "$LOG" | tail -3 | sed 's/^/    /'
  echo "  SPARSE  last 1:"
  grep SPARSE_STATS "$LOG" | tail -1 | sed 's/^/    /'
  echo "  KLT     last 1:"
  grep KLT_STATS    "$LOG" | tail -1 | sed 's/^/    /'

  python3 - "$LOG" "$tag" "$RESULT" <<'PY'
import re,sys
log,tag,out = sys.argv[1], sys.argv[2], sys.argv[3]
sp_t=kl_t=mt=sp=kl=ch=sr=nm=wi=lv=0.0
with open(log) as f:
    for line in f:
        m=re.search(r'\[SPARSE_STATS\][^\n]*mean_nmeas=([\d.]+) mean_chi2=([\d.]+) mean_time=([\d.]+)ms',line)
        if m: nm,ch,sp_t=float(m.group(1)),float(m.group(2)),float(m.group(3))
        m=re.search(r'\[KLT_STATS\][^\n]*mean_cost=([\d.]+)ms[^\n]*mean_win=([\d.]+)[^\n]*mean_lvls=([\d.]+)[^\n]*sparse_prior=([\d.]+)%',line)
        if m: kl_t,wi,lv,sp=float(m.group(1)),float(m.group(2)),float(m.group(3)),float(m.group(4))
        m=re.search(r'\[TRACKER_STATS\] frames=\d+ mean_total=([\d.]+)ms',line)
        if m: mt=float(m.group(1))
# last sparse_stats had succ; we don't have it; use a separate scan
m_succ=None
with open(log) as f:
    succs=[]
    for line in f:
        m=re.search(r'\[SPARSE_STATS\]\s+frames=\d+/(\d+)\s+success_rate=([\d.]+)%',line)
        if m: succs.append((int(m.group(1)),float(m.group(2))))
last=succs[-1] if succs else (0,0.0)
sparse_pct  = (sp_t/mt*100) if mt>0 else 0
klt_pct     = (kl_t/mt*100) if mt>0 else 0
with open(out,'a') as f:
    f.write(f'{tag},{sp_t:.3f},{kl_t:.3f},{mt:.3f},{sparse_pct:.2f},{klt_pct:.2f},{sp:.2f},{ch:.3f},{last[1]:.1f},{nm:.0f},{wi:.1f},{lv:.2f}\n')
PY
done < "$OUT/sweep.csv"

echo
echo "=== summary (rows tagged 'A'..'E') ==="
column -t -s, "$RESULT"
