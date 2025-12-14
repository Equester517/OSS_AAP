#!/bin/bash
# Auto-generated from elastic_band_smoother.txt
# Points: 126
# Left bound: 41
# Right bound: 38

source /opt/ros/humble/setup.bash
source /home/bskang/autoware/install/setup.bash

ros2 topic pub --once /planning/scenario_planning/lane_driving/motion_planning/path_optimizer/path_optimizer/input/path autoware_planning_msgs/msg/Path \
"{
  header: {
    stamp: {sec: 1764663506, nanosec: 87414201},
    frame_id: "map"
  },
  left_bound: [
    {x: 3701.0504707235805, y: 73664.1864260894, z: 19.45792682428737},
    {x: 3703.26698255952, y: 73665.34274578502, z: 19.45792682428737},
    {x: 3705.2935999935726, y: 73666.40000052936, z: 19.475},
    {x: 3706.813357735925, y: 73667.19287458003, z: 19.475},
    {x: 3708.5865480175053, y: 73668.11796712733, z: 19.475},
    {x: 3711.009600143996, y: 73669.38209962752, z: 19.486},
    {x: 3712.134120631213, y: 73669.96787840525, z: 19.486},
    {x: 3713.907889856648, y: 73670.89186012073, z: 19.486},
    {x: 3715.68165912208, y: 73671.81584185702, z: 19.486},
    {x: 3716.728200408281, y: 73672.36100015556, z: 19.501},
    {x: 3717.353499826975, y: 73672.68699947232, z: 19.506},
    {x: 3717.489700422855, y: 73672.75940052094, z: 19.507},
    {x: 3719.2278756164515, y: 73673.66545296011, z: 19.507},
    {x: 3721.0013889503653, y: 73674.58992598676, z: 19.507},
    {x: 3722.7749022789053, y: 73675.51439901062, z: 19.507},
    {x: 3724.5484156106704, y: 73676.43887203615, z: 19.507},
    {x: 3726.3219289595972, y: 73677.36334507063, z: 19.507},
    {x: 3728.0954423060302, y: 73678.2878181038, z: 19.507},
    {x: 3729.8689556522177, y: 73679.21229113685, z: 19.507},
    {x: 3731.241400213912, y: 73679.92770046229, z: 19.513},
    {x: 3731.695299885818, y: 73680.16310053878, z: 19.516},
    {x: 3732.048299561604, y: 73680.347200335, z: 19.519},
    {x: 3733.4154344135245, y: 73681.06070164824, z: 19.519},
    {x: 3735.1884905015986, y: 73681.98605136448, z: 19.519},
    {x: 3737.640800228808, y: 73683.26590039209, z: 19.555},
    {x: 3738.7357700104917, y: 73683.8364980574, z: 19.555},
    {x: 3740.5093983485053, y: 73684.7607502201, z: 19.555},
    {x: 3742.283026707483, y: 73685.68500239371, z: 19.555},
    {x: 3743.2358002740075, y: 73686.18150045397, z: 19.57},
    {x: 3745.829775631251, y: 73687.53387051797, z: 19.57},
    {x: 3747.6032279538977, y: 73688.45846057794, z: 19.57},
    {x: 3748.828199905227, y: 73689.09710018244, z: 19.588},
    {x: 3749.4484999856213, y: 73689.4200002728, z: 19.591},
    {x: 3751.1495891134973, y: 73690.30759729659, z: 19.591},
    {x: 3752.922713609969, y: 73691.23278105096, z: 19.591},
    {x: 3753.9252617074617, y: 73691.77146762339, z: 19.609},
    {x: 3754.7846160706736, y: 73692.24819313978, z: 19.5816},
    {x: 3756.4558237144097, y: 73693.11562695635, z: 19.5816},
    {x: 3758.391959745283, y: 73694.13911879133, z: 19.598},
    {x: 3759.013790369429, y: 73694.46150460589, z: 19.593},
    {x: 3763.7186695684322, y: 73696.87558993312, z: 19.575305205010288}
  ],
  right_bound: [
    {x: 3702.424927820335, y: 73661.55232518748, z: 19.530636533779248},
    {x: 3703.7918003739323, y: 73662.26569991885, z: 19.537},
    {x: 3706.414299258995, y: 73663.6338995981, z: 19.537},
    {x: 3708.1135001585353, y: 73664.52039992763, z: 19.553},
    {x: 3709.9606488453364, y: 73665.48393131161, z: 19.553},
    {x: 3711.7338966252305, y: 73666.40891363691, z: 19.553},
    {x: 3713.4136000607396, y: 73667.28510026308, z: 19.554},
    {x: 3715.2803947735338, y: 73668.25889791976, z: 19.554},
    {x: 3717.053635365614, y: 73669.18389386145, z: 19.554},
    {x: 3718.713599970273, y: 73670.04980034335, z: 19.552},
    {x: 3720.600018522074, y: 73671.03312534669, z: 19.552},
    {x: 3722.373532521383, y: 73671.9575970964, z: 19.552},
    {x: 3724.984300065262, y: 73673.31850027526, z: 19.589},
    {x: 3725.599600031972, y: 73673.63840049552, z: 19.589},
    {x: 3726.227399944619, y: 73673.96439974662, z: 19.589},
    {x: 3726.7895996998996, y: 73674.25710030599, z: 19.589},
    {x: 3727.695478166139, y: 73674.7298536059, z: 19.589},
    {x: 3729.4685505322177, y: 73675.65517178409, z: 19.589},
    {x: 3731.2416228996444, y: 73676.58048996299, z: 19.589},
    {x: 3733.0302002801327, y: 73677.51389978593, z: 19.589},
    {x: 3734.7877055641397, y: 73678.43074405745, z: 19.589},
    {x: 3736.5609228275903, y: 73679.3557848553, z: 19.589},
    {x: 3738.862199708179, y: 73680.55630046362, z: 19.611},
    {x: 3740.1074742148758, y: 73681.20620562698, z: 19.611},
    {x: 3741.8805289520496, y: 73682.13155776614, z: 19.611},
    {x: 3743.6535836695457, y: 73683.05690989504, z: 19.611},
    {x: 3744.691700363939, y: 73683.59869999532, z: 19.633},
    {x: 3747.199692537697, y: 73684.90717296653, z: 19.633},
    {x: 3749.053799931484, y: 73685.87450031145, z: 19.648},
    {x: 3750.7459664452094, y: 73686.7568827006, z: 19.648},
    {x: 3752.5193454393175, y: 73687.68161344135, z: 19.648},
    {x: 3753.5633384490993, y: 73688.21043161108, z: 19.66},
    {x: 3756.211325197037, y: 73689.58550347011, z: 19.6538},
    {x: 3757.9341410709103, y: 73690.45168060507, z: 19.664},
    {x: 3759.6345745772906, y: 73691.33837429598, z: 19.664},
    {x: 3760.738767648308, y: 73691.91416659231, z: 19.664},
    {x: 3762.4632911911344, y: 73692.83523730078, z: 19.653},
    {x: 3765.099283941396, y: 73694.2281585214, z: 19.64880356059236}
  ],
  points: [
    {
      pose: {
        position: {x: 3704.84063402042, y: 73664.48829601234, z: 19.50615814455585},
        orientation: {x: -0.00034882912244550745, y: 0.0014230994250604783, z: 0.23807127364540806, w: 0.9712465813433389}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3705.283955967406, y: 73664.7195224194, z: 19.507623377492227},
        orientation: {x: -0.0003488286586017933, y: 0.0014230995387880653, z: 0.2380709570733602, w: 0.9712466589412}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3705.727278065117, y: 73664.95074853746, z: 19.509088610428602},
        orientation: {x: -0.00024537875200787885, y: 0.0010010627726926688, z: 0.23807044603404476, w: 0.9712473425383177}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3706.170600467632, y: 73665.18197407115, z: 19.510119309605134},
        orientation: {x: -0.00024537778656211776, y: 0.0010010630043677134, z: 0.2380695104585013, w: 0.9712475718644824}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3706.613923315612, y: 73665.41319875076, z: 19.511150008776834},
        orientation: {x: -0.00024537641853778186, y: 0.0010010634600560466, z: 0.23806815617722069, w: 0.9712479038210279}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3707.0572467595616, y: 73665.64442216857, z: 19.512180707951853},
        orientation: {x: -0.0002453750162724609, y: 0.001001063569864769, z: 0.23806684815064044, w: 0.9712482244382485}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3707.5005709240254, y: 73665.87564444325, z: 19.51321140712687},
        orientation: {x: -0.0002453730879637912, y: 0.0010010641397525549, z: 0.23806495546123907, w: 0.9712486883614692}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3707.9438959495533, y: 73666.10686496919, z: 19.514242106303083},
        orientation: {x: -0.0002453709739085761, y: 0.001001064699833748, z: 0.23806289497039707, w: 0.9712491934106641}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3708.3872219386326, y: 73666.33808360498, z: 19.51527280547929},
        orientation: {x: -0.0002453686539558739, y: 0.001001065235992162, z: 0.23806065140146934, w: 0.9712497433291425}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3708.830549015599, y: 73666.56930020287, z: 19.516303504669683},
        orientation: {x: -0.0002453660986922444, y: 0.0010010659030651723, z: 0.2380581630991409, w: 0.971250353227425}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3709.273877255458, y: 73666.8005145178, z: 19.517334203848897},
        orientation: {x: -0.00012251368169703835, y: 0.0004998477759889161, z: 0.23805564088674844, w: 0.9712513819730699}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3709.717206781465, y: 73667.03172646445, z: 19.517848847071598},
        orientation: {x: -0.00012251226657432413, y: 0.0004998481925687495, z: 0.23805285984163968, w: 0.9712520636084876}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3710.1605375732083, y: 73667.26293584185, z: 19.5183634902943},
        orientation: {x: -0.0001225107918167448, y: 0.0004998485177976444, z: 0.23805001052563657, w: 0.9712527619684329}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3710.603869750688, y: 73667.4941426333, z: 19.51887813351567},
        orientation: {x: -0.0001225092521062967, y: 0.0004998488947606555, z: 0.23804701890786303, w: 0.9712534951966171}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3711.0472033524693, y: 73667.72534669367, z: 19.519392776736645},
        orientation: {x: -0.0001217758637215586, y: 0.0004968630960559687, z: 0.23804408492962706, w: 0.9712542159124559}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3711.4905383512723, y: 73667.95654807524, z: 19.519904345405227},
        orientation: {x: -0.00012177431707535847, y: 0.0004968634751178722, z: 0.2380410615894249, w: 0.9712549568960163}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3711.933874789445, y: 73668.18774669676, z: 19.52041591407381},
        orientation: {x: -0.0001217728350659123, y: 0.0004968638339972228, z: 0.2380381665589923, w: 0.9712556664230796}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3712.3772126058816, y: 73668.41894267536, z: 19.520927482738177},
        orientation: {x: -0.00012177135033029446, y: 0.0004968642148117385, z: 0.2380352565861317, w: 0.971256379603071}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3712.820551807675, y: 73668.65013599739, z: 19.521439051418987},
        orientation: {x: -0.00014143080639054432, y: 0.0005770878215859909, z: 0.2380325081498361, w: 0.9712570061682291}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3713.2638923480167, y: 73668.88132683864, z: 19.522033217605255},
        orientation: {x: -0.00014142927280663535, y: 0.0005770882947947779, z: 0.23802988919518722, w: 0.9712576480093157}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3713.7072340645855, y: 73669.1125152522, z: 19.522627383791527},
        orientation: {x: -0.00014142780976029686, y: 0.0005770885897606753, z: 0.23802745158359032, w: 0.9712582454010652}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3714.1505769870328, y: 73669.3437014641, z: 19.52322154997693},
        orientation: {x: -0.0001414264717011482, y: 0.0005770889660021753, z: 0.2380251807858177, w: 0.9712588019054227}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3714.5939209701914, y: 73669.57488559233, z: 19.523815716182053},
        orientation: {x: -0.00015905310472937655, y: 0.0006490200094817481, z: 0.23802316798166398, w: 0.9712592470494733}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3715.037265923967, y: 73669.80606790242, z: 19.524483941799815},
        orientation: {x: -0.00015905191674117224, y: 0.0006490203241069643, z: 0.23802138202912962, w: 0.9712596847249708}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3715.4806117127964, y: 73670.03724857417, z: 19.525152167417573},
        orientation: {x: -0.00015905086096172898, y: 0.0006490206548818963, z: 0.2380197771304003, w: 0.9712600780274085}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3715.9239582201926, y: 73670.26842775707, z: 19.525820393036827},
        orientation: {x: -0.00015905016986103397, y: 0.0006490206597445919, z: 0.23801879980772356, w: 0.9712603175324922}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3716.3673052988365, y: 73670.49960610301, z: 19.52648861865608},
        orientation: {x: -0.00015904950300514798, y: 0.0006490209071137522, z: 0.2380177728143165, w: 0.9712605692087114}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3716.8106528133617, y: 73670.73078348374, z: 19.52715684427698},
        orientation: {x: -0.00015904911000920737, y: 0.0006490210250717234, z: 0.23801717720537324, w: 0.9712607151688344}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3717.254000611425, y: 73670.96196032072, z: 19.52782506991892},
        orientation: {x: -0.00018188090634381016, y: 0.0007421901500971417, z: 0.23801687448373268, w: 0.9712607186199347}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3717.6973485458034, y: 73671.19313689627, z: 19.528589221674725},
        orientation: {x: -0.00018188096644322486, y: 0.0007421902952355172, z: 0.23801690476836274, w: 0.9712607111982694}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3718.1406964657667, y: 73671.42431349946, z: 19.529353373585806},
        orientation: {x: -0.00034796453149255676, y: 0.0014199149237167673, z: 0.23801706747676196, w: 0.9712598716883432}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3718.584044220277, y: 73671.65549041996, z: 19.530815307718797},
        orientation: {x: -0.00034796548890670114, y: 0.001419914689084528, z: 0.23801772237494606, w: 0.9712597111986769}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3719.027391663036, y: 73671.88666793834, z: 19.53227724185179},
        orientation: {x: -0.0003479670320601514, y: 0.0014199141120095858, z: 0.23801880938705208, w: 0.9712594448142212}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3719.470738646047, y: 73672.11784647917, z: 19.533739175981893},
        orientation: {x: -0.00034796862744596155, y: 0.001419914109419225, z: 0.23801983925819423, w: 0.9712591924307639}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3719.914085023369, y: 73672.34902590002, z: 19.535201110108588},
        orientation: {x: -0.0003442846878710087, y: 0.0014048708468109314, z: 0.23802154667294423, w: 0.9712587971933004}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3720.3574306491505, y: 73672.58020690535, z: 19.536647556507113},
        orientation: {x: -0.00034428741738821864, y: 0.0014048701845807675, z: 0.2380234326593205, w: 0.97125833500208}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3720.8007753751267, y: 73672.8113896314, z: 19.538094002905638},
        orientation: {x: -0.00034429069287345693, y: 0.0014048695711353016, z: 0.23802566691775376, w: 0.9712577874559182}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3721.244118979767, y: 73673.04257436708, z: 19.53954044929969},
        orientation: {x: -0.0003442941205377511, y: 0.0014048682487336388, z: 0.2380281137362111, w: 0.9712571878127562}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3721.687461537014, y: 73673.27376139775, z: 19.540986895608334},
        orientation: {x: -0.000256968709590365, y: 0.0010485295734767824, z: 0.23803109370920844, w: 0.9712569345854956}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3722.130802669409, y: 73673.5049510176, z: 19.542066456287028},
        orientation: {x: -0.00025697213830207683, y: 0.0010485287331678313, z: 0.23803426974014932, w: 0.9712561562132257}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3722.57414228981, y: 73673.73614353692, z: 19.543146016965725},
        orientation: {x: -0.0002569758780127421, y: 0.0010485278166135093, z: 0.23803773385341323, w: 0.9712553072260391}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3723.0174802610363, y: 73673.9673392187, z: 19.544225577644387},
        orientation: {x: -0.00025698000388495195, y: 0.001048526805274759, z: 0.23804155569684735, w: 0.9712543705509081}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3723.4608164127662, y: 73674.1985383895, z: 19.545305138322917},
        orientation: {x: -0.00025217126274343246, y: 0.001028887867592625, z: 0.23804556969960888, w: 0.9712534090266036}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3723.9041506559242, y: 73674.42974122002, z: 19.546364479801316},
        orientation: {x: -0.00025217587608688017, y: 0.0010288867368816082, z: 0.2380499246230907, w: 0.9712523416632524}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3724.3474828257263, y: 73674.66094802621, z: 19.54742382127972},
        orientation: {x: -0.00025218056894105834, y: 0.0010288857607930928, z: 0.23805431659732418, w: 0.9712512651978543}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3724.7908129044836, y: 73674.89215884186, z: 19.548483162927226},
        orientation: {x: -0.0002521854820766548, y: 0.001028884343560017, z: 0.23805900100851135, w: 0.971250117033877}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3725.2341407529293, y: 73675.12337393392, z: 19.549542504367853},
        orientation: {x: -2.3046563940891736e-05, y: 9.402512095141282e-05, z: 0.2380636638527799, w: 0.9712495470172036}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3725.677466444356, y: 73675.35459316177, z: 19.549639312775252},
        orientation: {x: -2.3047020580539143e-05, y: 9.402500901896096e-05, z: 0.2380683808044476, w: 0.9712483908296997}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3726.1207898898924, y: 73675.5858166957, z: 19.549736121182647},
        orientation: {x: -2.3047434158612942e-05, y: 9.402487965212796e-05, z: 0.23807271979166109, w: 0.9712473272648625}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3726.564111269464, y: 73675.81704419067, z: 19.549832929562857},
        orientation: {x: -2.30478584503539e-05, y: 9.402477568783313e-05, z: 0.23807710249739508, w: 0.9712462529629562}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3727.0074305621984, y: 73676.04827568655, z: 19.549929737943103},
        orientation: {x: -2.3074381402819957e-05, y: 9.413140173964628e-05, z: 0.23808086144751187, w: 0.9712453315302282}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3727.450748065089, y: 73676.27951061394, z: 19.550026656198018},
        orientation: {x: -2.3074735553104648e-05, y: 9.413131492949754e-05, z: 0.23808451555313898, w: 0.9712444357939352}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3727.894063828007, y: 73676.51074887709, z: 19.550123574452932},
        orientation: {x: -2.3075034119906687e-05, y: 9.413126313793032e-05, z: 0.23808754510407806, w: 0.9712436931446}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3728.337378053098, y: 73676.74198985618, z: 19.55022049270781},
        orientation: {x: -2.307522089897594e-05, y: 9.413121979617079e-05, z: 0.2380894664518787, w: 0.9712432221495846}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3728.780691553769, y: 73676.97323268859, z: 19.550317411006702},
        orientation: {x: -6.433593926944376e-05, y: 0.00026244471135201903, z: 0.23809134107410382, w: 0.9712427298513978}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3729.224004141022, y: 73677.20447723026, z: 19.550587626441708},
        orientation: {x: -6.433600042017536e-05, y: 0.0002624447943698047, z: 0.2380914835042997, w: 0.9712426949358903}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3729.6673165042844, y: 73677.4357218205, z: 19.550857841876713},
        orientation: {x: -6.433576917264925e-05, y: 0.00026244481097499, z: 0.23809066201695656, w: 0.9712428963158276}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3730.1106293539624, y: 73677.6669657105, z: 19.55112805733084},
        orientation: {x: -6.433562162262168e-05, y: 0.00026244486868099993, z: 0.2380900975414231, w: 0.9712430346912838}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3730.5539424381177, y: 73677.8982090673, z: 19.551398272784965},
        orientation: {x: -6.433462835768916e-05, y: 0.00026244514249793786, z: 0.23808639575992968, w: 0.9712439421369118}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3730.997257302112, y: 73678.12944905375, z: 19.551668488278995},
        orientation: {x: -6.433347132095246e-05, y: 0.00026244542609013213, z: 0.238082113886601, w: 0.9712449917661817}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3731.440574205051, y: 73678.3606851314, z: 19.551938703773022},
        orientation: {x: -6.433174541225982e-05, y: 0.0002624458959980499, z: 0.23807568664690254, w: 0.9712465672583785}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3731.8838941682984, y: 73678.59191534166, z: 19.5522089193125},
        orientation: {x: -6.433028509804926e-05, y: 0.00026244904487639816, z: 0.2380678941990566, w: 0.9712484773392132}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3732.3272178419807, y: 73678.82313843833, z: 19.552479137562724},
        orientation: {x: -0.0005297460197241192, y: 0.0021613099122221825, z: 0.23805706341344837, w: 0.9712486204199126}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3732.77054639584, y: 73679.05435217766, z: 19.554704438653634},
        orientation: {x: -0.000529719406457523, y: 0.002161316588055301, z: 0.23804508805902894, w: 0.9712515555503248}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3733.213880651423, y: 73679.28555498469, z: 19.556929739894155},
        orientation: {x: -0.0005483751039726727, y: 0.0022375810523808785, z: 0.23803028763833778, w: 0.9712549998237279}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3733.657221932784, y: 73679.51674431865, z: 19.559233556016988},
        orientation: {x: -0.0005483362943832688, y: 0.00223759056158938, z: 0.23801344193460408, w: 0.9712591281291671}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3734.100571233795, y: 73679.74791827382, z: 19.561537372139817},
        orientation: {x: -0.0005482902866421698, y: 0.0022376013103601704, z: 0.23799352429700352, w: 0.9712640088620751}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3734.5439300154408, y: 73679.9790740451, z: 19.56384118775068},
        orientation: {x: -0.0005482387711268621, y: 0.0022376139302267635, z: 0.23797116348510636, w: 0.9712694877672694}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3734.987299440586, y: 73680.2102094021, z: 19.56614500336154},
        orientation: {x: -0.0005481795790081019, y: 0.0022376281703450594, z: 0.23794549652586924, w: 0.9712757760813239}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3735.430681080622, y: 73680.44132132544, z: 19.568448818715538},
        orientation: {x: -0.000548113947342004, y: 0.0022376437554858446, z: 0.23791705752086914, w: 0.9712827426975812}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3735.8740762544567, y: 73680.67240728436, z: 19.570752633594697},
        orientation: {x: -0.0004674022140480474, y: 0.0019084140702288, z: 0.23788531711605526, w: 0.9712912618733377}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3736.3174866117497, y: 73680.90346410613, z: 19.572717462798906},
        orientation: {x: -0.0004673335027926891, y: 0.0019084308935085597, z: 0.23785034684398737, w: 0.9712998260048834}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3736.760913606575, y: 73681.13448899888, z: 19.574682292003114},
        orientation: {x: -0.00046725775131348327, y: 0.0019084493144250955, z: 0.23781180794941484, w: 0.9713092625375497}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3737.2043589321147, y: 73681.36547870236, z: 19.57664712107982},
        orientation: {x: -0.00046717554067169043, y: 0.0019084693836737665, z: 0.23776997337486958, w: 0.9713195042070036}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3737.647824154572, y: 73681.5964302071, z: 19.578611950105913},
        orientation: {x: -0.0004476687704760593, y: 0.0018291521543109125, z: 0.2377245938818285, w: 0.9713307733510701}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3738.0913109715525, y: 73681.82734023953, z: 19.58049509711492},
        orientation: {x: -0.00044757669290577024, y: 0.0018291746814450682, z: 0.23767569880964506, w: 0.9713427386820965}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3738.5348210346233, y: 73682.05820562305, z: 19.582378244123927},
        orientation: {x: -0.0004474780074898497, y: 0.0018291986364034028, z: 0.23762331731028175, w: 0.9713555542983418}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3738.978355993475, y: 73682.28902317068, z: 19.584261390943997},
        orientation: {x: -0.00044737260324964856, y: 0.0018292244120011677, z: 0.237567345486108, w: 0.9713692450116426}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3739.421917551148, y: 73682.51978960281, z: 19.586144537764067},
        orientation: {x: -0.0003795877056868632, y: 0.0015524745905081089, z: 0.23750823734534238, w: 0.9713842097383136}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3739.865507244416, y: 73682.75050194364, z: 19.5877427504393},
        orientation: {x: -0.0003794872119709292, y: 0.0015524988744991797, z: 0.23744539932604508, w: 0.9713995717909266}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3740.30912678444, y: 73682.9811568929, z: 19.5893409628444},
        orientation: {x: -0.00037938208409739565, y: 0.0015525240155611479, z: 0.237379700425857, w: 0.9714156286390218}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3740.752777518701, y: 73683.211751833, z: 19.590939174708243},
        orientation: {x: -0.0003792714479400313, y: 0.0015525510413154386, z: 0.23731047609952846, w: 0.9714325420078576}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3741.1964611146745, y: 73683.44228354187, z: 19.592537386572086},
        orientation: {x: -0.0003396195413701337, y: 0.0013906796285613106, z: 0.23723903212924735, w: 0.9714502520989512}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3741.640178641606, y: 73683.67274993207, z: 19.59396893952864},
        orientation: {x: -0.00033951227673792916, y: 0.0013907051045854726, z: 0.23716421815987476, w: 0.9714685194566736}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3742.0839316619576, y: 73683.90314797715, z: 19.595400491795143},
        orientation: {x: -0.00033940283434007727, y: 0.0013907307988317125, z: 0.2370879318474076, w: 0.971487140031138}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3742.527720860305, y: 73684.13347632652, z: 19.59683204306807},
        orientation: {x: -0.0003392894141869569, y: 0.0013907584703461355, z: 0.2370087033815766, w: 0.9715064720293707}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3742.971547621691, y: 73684.36373228868, z: 19.59826359434099},
        orientation: {x: -0.00034411043994479523, y: 0.0014110222544401923, z: 0.2369290715614257, w: 0.9715258646342069}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3743.4154121196857, y: 73684.59391549604, z: 19.59971597485554},
        orientation: {x: -0.00034399137805799683, y: 0.0014110515310272083, z: 0.23684705536548925, w: 0.9715458625140687}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3743.8593154755263, y: 73684.82402375905, z: 19.60116835561047},
        orientation: {x: -0.00034387338586891047, y: 0.0014110788925754967, z: 0.23676603604939458, w: 0.9715656101267224}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3744.303257203232, y: 73685.05405798348, z: 19.60262073500725},
        orientation: {x: -0.0003437533563332822, y: 0.001411108138963666, z: 0.2366833923165624, w: 0.9715857462977596}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3744.747238058298, y: 73685.28401667965, z: 19.604073114404006},
        orientation: {x: -0.00034951486434268774, y: 0.0014352723104638864, z: 0.236603497516396, w: 0.9716051681602786}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3745.191256723288, y: 73685.51390236343, z: 19.60555033529227},
        orientation: {x: -0.0003493961051541275, y: 0.0014353018738092556, z: 0.23652300279128474, w: 0.9716247665541912}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3745.6353134712185, y: 73685.74371447273, z: 19.607027556806248},
        orientation: {x: -0.00034928416447036723, y: 0.0014353276675718228, z: 0.23644745049154625, w: 0.9716431551712321}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3746.079405954889, y: 73685.9734575222, z: 19.608504776915495},
        orientation: {x: -0.00034917355945582275, y: 0.0014353545867132013, z: 0.23637257530810502, w: 0.9716613728440839}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3746.5235338387292, y: 73686.20313212447, z: 19.609981997024693},
        orientation: {x: -0.0003476727337426654, y: 0.001429615862532676, z: 0.23630533481090052, w: 0.9716777367327422}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3746.967693509251, y: 73686.43274525579, z: 19.611453286230688},
        orientation: {x: -0.0003475769780498514, y: 0.001429638876166656, z: 0.23624029396183976, w: 0.9716935519142604}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3747.4118839112866, y: 73686.6622989227, z: 19.612924575163227},
        orientation: {x: -0.00034749768981672506, y: 0.0014296589210256734, z: 0.23618628334116787, w: 0.9717066815053865}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3747.856099832587, y: 73686.8918032104, z: 19.614395864854888},
        orientation: {x: -0.00034742363634206473, y: 0.0014296769316681306, z: 0.23613594885164932, w: 0.9717189145944517}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3748.3003395242386, y: 73687.12126147434, z: 19.61586715454659},
        orientation: {x: -0.00034378240611711546, y: 0.0014149611471467084, z: 0.23609369400317337, w: 0.9717292047430421}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3748.744599174259, y: 73687.35068109859, z: 19.61732328465173},
        orientation: {x: -0.0003437602761618813, y: 0.001414965419612951, z: 0.23607867012970415, w: 0.9717328548585378}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3749.1888659171523, y: 73687.58008698486, z: 19.61877941368069},
        orientation: {x: -0.0003437345546132746, y: 0.0014149677509408683, z: 0.23606162288232196, w: 0.9717369962675004}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3749.633140709093, y: 73687.80947728353, z: 19.620235538904062},
        orientation: {x: -0.00034373317272796294, y: 0.0014149680855800155, z: 0.23606067403155656, w: 0.9717372267689398}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3750.077415949222, y: 73688.03886671469, z: 19.621691664127106},
        orientation: {x: -0.0002807016514936549, y: 0.0011553580378720044, z: 0.23608825705535869, w: 0.9717308893078114}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3750.5216782040534, y: 73688.26828129034, z: 19.622880634942575},
        orientation: {x: -0.00028072578886435807, y: 0.0011553235633489426, z: 0.23611407908327173, w: 0.971724615351054}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3750.9659282684756, y: 73688.49771947815, z: 19.624069577964423},
        orientation: {x: -0.00028083231446382126, y: 0.0011554069590828278, z: 0.23618258089070537, w: 0.9717079677823866}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3751.4101459723706, y: 73688.72722029538, z: 19.62525862716034},
        orientation: {x: -0.0002809143350309891, y: 0.001155326146214837, z: 0.23626331417937224, w: 0.9716883413325549}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3751.854325548621, y: 73688.95679493234, z: 19.626447617244107},
        orientation: {x: 4.258437572139642e-05, y: -0.00017504854656881432, z: 0.23637777640875188, w: 0.9716612137801073}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3752.298451082934, y: 73689.1864740514, z: 19.62626746335348},
        orientation: {x: 4.2608936037750615e-05, y: -0.00017504255382508373, z: 0.2365141265766922, w: 0.9716280334954562}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3752.742512168502, y: 73689.41627782462, z: 19.62608730946625},
        orientation: {x: 4.263797560595162e-05, y: -0.0001750245393450832, z: 0.23668928845612144, w: 0.971585378790021}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3753.186490330498, y: 73689.6462416753, z: 19.625907166225957},
        orientation: {x: 4.2674473313336575e-05, y: -0.000175015611664127, z: 0.23689193365093086, w: 0.9715359897191396}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3753.6303725767243, y: 73689.8763907408, z: 19.625727022995548},
        orientation: {x: 4.271404702912784e-05, y: -0.000174984925927587, z: 0.23713850904588582, w: 0.9714758335045088}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3754.074137878688, y: 73690.10676506668, z: 19.625546900221302},
        orientation: {x: 4.276441791098848e-05, y: -0.00017497258313903903, z: 0.23741820711790187, w: 0.9714075161767695}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3754.5177705323786, y: 73690.33739493653, z: 19.625366777451198},
        orientation: {x: 4.2821383193020076e-05, y: -0.00017494961511876203, z: 0.23774604939668337, w: 0.9713273307980328}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3754.961247316748, y: 73690.56832416408, z: 19.62518666349425},
        orientation: {x: 4.288737959447223e-05, y: -0.0001749333991323962, z: 0.23811252731020596, w: 0.9712375568813868}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3755.404549832213, y: 73690.79958804937, z: 19.625006549535257},
        orientation: {x: 4.2963980004200954e-05, y: -0.0001749205544414292, z: 0.238530160143146, w: 0.9711350731278225}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3755.847653159718, y: 73691.03123304396, z: 19.624826429846667},
        orientation: {x: 4.3046995401260584e-05, y: -0.00017490007193021536, z: 0.2389911422800044, w: 0.9710217306881549}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3756.2905365249685, y: 73691.26329871388, z: 19.624646310159665},
        orientation: {x: 4.3138031332761946e-05, y: -0.00017487109785991643, z: 0.23950500840289318, w: 0.9708951120018782}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3756.7331738413445, y: 73691.4958329462, z: 19.62446619689259},
        orientation: {x: 4.323911817800502e-05, y: -0.00017484604237084892, z: 0.24006636253909672, w: 0.9707564623202295}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3757.175542161615, y: 73691.7288790217, z: 19.624286083624025},
        orientation: {x: 4.3355007594740836e-05, y: -0.00017483904526048512, z: 0.240681639492334, w: 0.9706040984680282}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3757.61761448211, y: 73691.96248560124, z: 19.624105949374734},
        orientation: {x: 4.3474901257799996e-05, y: -0.0001748091622858567, z: 0.24134736093407108, w: 0.9704387764933184}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3758.0593661654143, y: 73692.19669857046, z: 19.623925815125443},
        orientation: {x: 4.361684273534375e-05, y: -0.00017478215844675163, z: 0.2421244373330722, w: 0.9702451877720968}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3758.5007417516017, y: 73692.43161855843, z: 19.62374567293098},
        orientation: {x: 4.373252023204835e-05, y: -0.00017474286799979362, z: 0.2427801565946195, w: 0.9700813177854157}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3758.9417999154102, y: 73692.66713525797, z: 19.62356554058834},
        orientation: {x: -8.521865257113408e-05, y: 0.00033961389848762913, z: 0.243382709885571, w: 0.9699302727149706}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3759.3825668236645, y: 73692.90320064784, z: 19.623915684857625},
        orientation: {x: -8.572330482062263e-05, y: 0.0003394957033907621, z: 0.24481799686788255, w: 0.9695689897082002}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3759.8226298727986, y: 73693.14056802585, z: 19.624265835188577},
        orientation: {x: -0.00012912891593562943, y: 0.0005054584179937177, z: 0.24751943882978092, w: 0.9688827871517283}
      },
      longitudinal_velocity_mps: 8.333333015441895,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    },
    {
      pose: {
        position: {x: 3760.0371, y: 73693.2578, z: 19.624520858405422},
        orientation: {x: -0.00012912891593562943, y: 0.0005054584179937177, z: 0.24751943882978092, w: 0.9688827871517283}
      },
      longitudinal_velocity_mps: 0.0,
      lateral_velocity_mps: 0.0,
      heading_rate_rps: 0.0,
      is_final: true
    }
  ]
}"