# lidar 建图结果通过CAD 转到gj坐标系下，然后评估绝对轨迹误差
evo_ape tum \
  /home/udeer/data/InsGj/0408/0414check/lidarpose_CAD_inggongji_tumformat_dx-0.260000_dy0.850000 \
  /home/udeer/data/InsGj/0408/0414check/_udeer_drivers_gnss_gjins_ins_tumformat \
  -r trans_part --plot --plot_mode xy -va \
  --t_end 1744082502.95032

# lidar 定位结果和 建图结果进行evo核对
evo_ape tum \
  /home/udeer/data/InsGj/0408/mapping_result/mapping_result_tum_format \
  /home/udeer/data/InsGj/lidarlocalizer/fusion_tum.txt \
  -r trans_part --plot --plot_mode xy -va

evo_ape tum \
  /home/udeer/data/InsGj/0408/0417result/fusion_tum_ingj_fromI.txt \
  /home/udeer/data/InsGj/0408/0417result/_udeer_drivers_gnss_gjins_ins_tumformat \
  -r trans_part --plot --plot_mode xy -v

evo_ape tum \
  \
  -r trans_part --plot --plot_mode xy -v
