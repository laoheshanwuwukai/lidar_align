evo_rpe tum mapping_result_tum_format _udeer_drivers_gnss_gjins_ins_tumformat -va -r full --plot --plot_mode xyz

evo_rpe tum mapping_result_tum_format _udeer_drivers_gnss_gjins_ins_tumformat -va -r full --plot --plot_mode xyz --t_end 1744082548.45024

evo_rpe tum mapping_result_tum_format _udeer_drivers_gnss_gjins_ins_tumformat -va -r full --plot --plot_mode xyz --t_end 1744082502.95032

# lidar 建图结果通过CAD值转换到gj坐标系下，然后评估绝对轨迹 ， 截止时间先到1744082502.95032
evo_ape tum /home/udeer/data/InsGj/0408/mapping_result/mapping_result_in_gj_rpy /home/udeer/data/InsGj/0408/_udeer_drivers_gnss_gjins_ins_tumformat \
  -va -r full --plot --plot_mode xyz --t_end 1744082502.95032
