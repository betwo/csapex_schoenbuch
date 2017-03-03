#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
adapters:
  []
connections:
  - uuid: csapex::EvaOptimizer_1:|:out_y_icr_r
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_r
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_x_icr
    targets:
      - csapex::ParamEstimator_0:|:in_x_icr
    types:
      - default
  - uuid: csapex::MarkerMessageDetector_0:|:event_0
    targets:
      - csapex::EvaOptimizer_1:|:slot_4
    types:
      - default
  - types:
      - default
    uuid: csapex::EvaOptimizer_1:|:event_0
    targets:
      - csapex::FileImporter_0:|:slot_1
  - targets:
      - csapex::ParamEstimator_0:|:in_y_icr_l
    types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_y_icr_l
  - uuid: csapex::EvaOptimizer_1:|:event_4
    targets:
      - csapex::FileImporter_0:|:slot_0
      - csapex::PillarLocalization_0:|:slot_0
      - csapex::ParamEstimator_0:|:slot_0
      - csapex::PoseDifference_0:|:slot_0
    types:
      - default
      - default
      - default
      - default
  - uuid: csapex::FileImporter_0:|:out_2
    targets:
      - csapex::PassThrough_0:|:in_0
      - csapex::ExportRos_6:|:in_0
    types:
      - default
      - default
  - types:
      - default
    uuid: csapex::EvaOptimizer_1:|:event_2
    targets:
      - csapex::FileImporter_0:|:slot_2
  - targets:
      - csapex::ParamEstimator_0:|:in_alpha_l
    types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_alpha_l
  - uuid: csapex::FileImporter_0:|:out_3
    targets:
      - csapex::ParamEstimator_0:|:in_0
    types:
      - default
  - types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_1
    targets:
      - csapex::TextDisplay_12:|:in_0
  - uuid: csapex::PillarLocalization_0:|:out_1
    targets:
      - csapex::PoseDifference_0:|:in_1
      - csapex::TransformToOdometry_5:|:in_0
    types:
      - default
      - default
  - targets:
      - csapex::EvaOptimizer_1:|:slot_4
      - csapex::MarkerMessageDetector_0:|:in_0
    types:
      - default
      - default
    uuid: csapex::PoseDifference_0:|:out_1
  - types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_alpha_r
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_r
  - uuid: csapex::TransformToOdometry_1:|:out_0
    targets:
      - csapex::ExportRos_8:|:in_0
    types:
      - default
  - uuid: csapex::PillarLocalization_0:|:out_0
    targets:
      - csapex::TransformInverter_5:|:in_0
      - csapex::TransformToOdometry_2:|:in_0
    types:
      - default
      - default
  - targets:
      - csapex::ExportRos_12:|:in_0
    types:
      - default
    uuid: csapex::TransformToOdometry_5:|:out_0
  - types:
      - default
    uuid: csapex::TransformToOdometry_2:|:out_0
    targets:
      - csapex::ExportRos_7:|:in_0
  - uuid: csapex::PassThrough_0:|:out_0
    targets:
      - csapex::PillarLocalization_0:|:in_0
      - csapex::ExportRos_5:|:in_0
    types:
      - default
      - default
  - types:
      - default
      - default
    uuid: csapex::ParamEstimator_0:|:out_0
    targets:
      - csapex::PoseDifference_0:|:in_0
      - csapex::ChangeTransformFrames_0:|:in_0
  - uuid: csapex::EvaOptimizer_1:|:out_0
    targets:
      - csapex::TextDisplay_13:|:in_0
    types:
      - default
  - types:
      - default
      - default
    uuid: csapex::FileImporter_0:|:out_0
    targets:
      - csapex::ParamEstimator_0:|:in_1
      - csapex::ExportRos_11:|:in_0
  - uuid: csapex::TransformInverter_5:|:out_0
    targets:
      - csapex::TransformPublisher_1:|:in_0
    types:
      - default
  - uuid: csapex::ChangeTransformFrames_0:|:out_0
    targets:
      - csapex::TransformToOdometry_1:|:in_0
    types:
      - default
uuid_map:
  csapex::FileImporter: 1
  csapex::PointCloudToDepthImage: 1
  csapex::TimePlot: 2
  csapex::ChangeTransformFrames: 5
  csapex::StaticTransform: 3
  csapex::TextDisplay: 19
  csapex::ExportRos: 13
  csapex::TransformToPose: 4
  csapex::SignTransformPublisher: 1
  csapex::PassThrough: 1
  csapex::OutputDisplay: 2
  csapex::OdometryToTransform: 1
  csapex::IntensityFilter: 1
  csapex::MarkerMessageDetector: 2
  csapex::ParamEstimator: 1
  csapex::ClusterPointcloud: 1
  csapex::TransformInverter: 6
  csapex::EvaOptimizer: 2
  csapex::PillarLocalization: 1
  csapex::PoseDifference: 1
  csapex::LabelClusteredPointCloud: 2
  csapex::TransformPublisher: 4
  csapex::TransformToOdometry: 6
  csapex::TransformCloud: 1
  csapex::PointCloudToIntensityImage: 1
  csapex::CloudRenderer: 2
  csapex::ExtractMeasures: 1
  csapex::ImportRos: 1
  csapex::SignalLight: 1
  csapex::SpinelloClustering: 1
  csapex::ColorPointCloud: 3
  csapex::Delay: 2
  csapex::PointCount: 1
nodes:
  - flipped: false
    exec_mode: 1
    logger_level: 1
    max_frequency: 0
    type: csapex::FileImporter
    label: csapex::FileImporter_0
    muted: false
    uuid: csapex::FileImporter_0
    pos:
      - -558
      - 885
    z: 842
    minimized: false
    enabled: true
    state:
      params:
        output_count:
          type: value
          name: output_count
          int: 4
        playback/resend:
          type: value
          name: playback/resend
          bool: false
        directory/loop:
          type: value
          name: directory/loop
          bool: true
        directory/quit on end:
          type: value
          name: directory/quit on end
          bool: false
        directory/filter:
          type: value
          name: directory/filter
          string: .*
        playback/immediate:
          bool: true
          type: value
          name: playback/immediate
        directory/latch:
          type: value
          name: directory/latch
          bool: false
        playback/frequency:
          step: 0.5
          min: 1
          max: 256
          name: playback/frequency
          double: 23.5
          type: range
        directory/sort_numerically:
          name: directory/sort_numerically
          bool: false
          type: value
        bag/forward_tf:
          type: value
          name: bag/forward_tf
          bool: true
        bag/latch:
          name: bag/latch
          bool: false
          type: value
        bag/frame:
          max: 3839
          type: range
          name: bag/frame
          step: 1
          int: 1419
          min: 0
        topic:
          txt: topic
          string: /wheel_velocities
          type: set
          name: topic
        directory/current_file:
          name: directory/current_file
          string: ""
          type: value
        cache:
          type: value
          name: cache
          bool: false
        directory/show parameters:
          type: value
          name: directory/show parameters
          bool: false
        directory/play:
          type: value
          name: directory/play
          bool: false
        bag/loop:
          type: value
          name: bag/loop
          bool: false
        import directory:
          type: value
          name: import directory
          bool: false
        bag/publish tf:
          bool: true
          name: bag/publish tf
          type: value
        directory:
          name: directory
          value: ""
          type: path
        directory/current:
          type: range
          step: 1
          max: 0
          name: directory/current
          int: 0
          min: 0
        path:
          value: /home/buck/data/schoenbuch/coarse_gravel_2016-03-07-16-34-32.bag
          type: path
          name: path
        split_container_messages:
          name: split_container_messages
          bool: false
          type: value
        bag/publish clock:
          name: bag/publish clock
          bool: true
          type: value
        bag/play:
          type: value
          name: bag/play
          bool: true
        recursive import:
          name: recursive import
          bool: false
          type: value
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
  - muted: false
    enabled: true
    flipped: false
    type: csapex::PassThrough
    minimized: false
    z: 817
    pos:
      - 20
      - 814
    color:
      - -1
      - -1
      - -1
    uuid: csapex::PassThrough_0
    max_frequency: 0
    label: csapex::PassThrough_0
    state:
      persistent_params:
        []
      params:
        keep organized:
          type: value
          name: keep organized
          bool: true
        interval:
          step: 0.01
          max: 1024
          min: 0
          name: interval
          double:
            - 148.21
            - 1023.98
          type: interval
        field:
          string: intensity
          type: set
          name: field
          txt: intensity
    exec_mode: 1
    logger_level: 1
  - state:
      persistent_params:
        []
      params:
        distance/2:
          type: range
          name: distance/2
          step: 0.001
          double: 8.358000000000001
          min: 0.01
          max: 20
        clustering/max distance vertically:
          type: range
          name: clustering/max distance vertically
          step: 0.001
          double: 0.026
          max: 0.1
          min: 0
        pillar/min points:
          step: 1
          int: 4
          max: 512
          min: 0
          type: range
          name: pillar/min points
        clustering/max cluster size:
          min: 0
          max: 1024
          type: range
          name: clustering/max cluster size
          int: 50
          step: 1
        min pts:
          max: 100
          step: 1
          name: min pts
          int: 3
          min: 1
          type: range
        clustering/max distance on ring:
          double: 0.043
          min: 0
          type: range
          name: clustering/max distance on ring
          max: 0.1
          step: 0.001
        clustering/max cluster diameter:
          step: 0.01
          name: clustering/max cluster diameter
          double: 0.2
          max: 5
          min: 0
          type: range
        distance/1:
          min: 0.01
          max: 20
          step: 0.001
          type: range
          name: distance/1
          double: 9.832000000000001
        clustering/min cluster size:
          name: clustering/min cluster size
          int: 2
          min: 0
          max: 1024
          step: 1
          type: range
        undistortion:
          type: value
          name: undistortion
          bool: false
        distance/3:
          max: 20
          step: 0.001
          name: distance/3
          double: 5.572
          min: 0.01
          type: range
        pillar/min intensity:
          step: 0.1
          min: 0
          max: 1024
          type: range
          name: pillar/min intensity
          double: 140
        cell/resolution:
          max: 0.5
          step: 0.01
          type: range
          name: cell/resolution
          double: 0.05
          min: 0.01
        clustering/max distance euclidean:
          max: 1
          step: 0.01
          type: range
          name: clustering/max distance euclidean
          double: 0.5
          min: 0
        threshold:
          max: 1
          step: 0.001
          name: threshold
          double: 0.25
          min: 0
          type: range
        size:
          step: 1
          int: 28
          max: 50
          min: 1
          type: range
          name: size
        reset:
          type: trigger
          name: reset
        radius:
          step: 0.001
          min: 0.01
          max: 1
          type: range
          name: radius
          double: 0.055
        pillar/radius threshold:
          max: 1
          step: 0.001
          name: pillar/radius threshold
          double: 0.067
          min: 0
          type: range
        pillar/radius:
          name: pillar/radius
          double: 0.065
          min: 0.001
          max: 1
          step: 0.001
          type: range
    pos:
      - 650
      - 574
    color:
      - -1
      - -1
      - -1
    max_frequency: 0
    label: csapex::PillarLocalization_0
    enabled: true
    flipped: false
    minimized: false
    muted: true
    exec_mode: 1
    logger_level: 1
    type: csapex::PillarLocalization
    uuid: csapex::PillarLocalization_0
    z: 1093
  - type: csapex::ParamEstimator
    pos:
      - 347
      - 1217
    color:
      - -1
      - -1
      - -1
    enabled: true
    flipped: false
    uuid: csapex::ParamEstimator_0
    max_frequency: 0
    label: csapex::ParamEstimator_0
    muted: false
    minimized: false
    state:
      persistent_params:
        []
      params:
        alpha_l:
          name: alpha_l
          interactive: true
          double: 0.6883008229149283
          type: range
          min: 0.5
          step: 0.001
          max: 1
        y_icr_r:
          step: 0.001
          double: -0.7260210816393411
          max: -0.001
          min: -0.75
          type: range
          name: y_icr_r
          interactive: true
        y_icr_l:
          max: 0.75
          step: 0.001
          interactive: true
          double: 0.2203601531429508
          min: 0.001
          type: range
          name: y_icr_l
        x_icr:
          max: 0
          step: 0.001
          interactive: true
          double: -0.05722129531126471
          min: -0.1
          type: range
          name: x_icr
        reset:
          name: reset
          type: trigger
        alpha_r:
          max: 1
          step: 0.001
          name: alpha_r
          interactive: true
          double: 0.7476687187003221
          min: 0.5
          type: range
    z: 906
    exec_mode: 1
    logger_level: 1
  - flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
    muted: false
    enabled: true
    minimized: false
    z: 1090
    color:
      - -1
      - -1
      - -1
    label: csapex::PoseDifference_0
    pos:
      - 1116
      - 1144
    type: csapex::PoseDifference
    uuid: csapex::PoseDifference_0
    max_frequency: 0
  - muted: false
    enabled: true
    minimized: false
    color:
      - -1
      - -1
      - -1
    z: 1092
    label: csapex::EvaOptimizer_1
    pos:
      - -619
      - 41
    type: csapex::EvaOptimizer
    uuid: csapex::EvaOptimizer_1
    max_frequency: 0
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        - alpha_l
        - alpha_r
        - x_icr
        - y_icr_l
        - y_icr_r
      params:
        individuals/later_generations:
          type: range
          name: individuals/later_generations
          int: 15
          min: 10
          max: 30
          step: 1
        stop:
          type: trigger
          name: stop
        server name:
          type: value
          name: server name
          string: localhost
        x_icr:
          name: x_icr
          interactive: true
          type: range
          step: 0.001
          double: -0.05722129531126471
          min: -0.1
          max: 0
        perform_evaluation:
          type: value
          name: perform_evaluation
          bool: true
        individuals/individuals:
          type: range
          name: individuals/individuals
          int: 100
          min: 10
          max: 100
          step: 1
        generations:
          step: 1
          max: 1024
          name: generations
          int: -1
          min: -1
          type: range
        method:
          int: 1
          type: set
          name: method
          txt: Differential Evolution
        start:
          type: trigger
          name: start
        y_icr_l:
          interactive: true
          double: 0.2203601531429508
          min: 0.001
          type: range
          name: y_icr_l
          max: 0.75
          step: 0.001
        population:
          type: progress
          name: population
        generation:
          name: generation
          type: progress
        fitness level:
          name: fitness level
          type: progress
        alpha_r:
          name: alpha_r
          interactive: true
          double: 0.7476687187003221
          type: range
          min: 0.5
          step: 0.001
          max: 1
        server port:
          type: value
          name: server port
          string: 2342
        set best:
          type: trigger
          name: set best
        y_icr_r:
          step: 0.001
          double: -0.7260210816393411
          min: -0.75
          max: -0.001
          type: range
          name: y_icr_r
          interactive: true
        finish:
          type: trigger
          name: finish
        finish_immediately:
          type: value
          name: finish_immediately
          bool: false
        individuals/target:
          step: 1
          max: 30
          type: range
          name: individuals/target
          int: 30
          min: 30
        alpha_l:
          max: 1
          step: 0.001
          interactive: true
          double: 0.6883008229149283
          min: 0.5
          type: range
          name: alpha_l
  - muted: false
    enabled: true
    state:
      params:
        type:
          type: bitset
          name: type
          int: 1
      persistent_params:
        []
    uuid: csapex::MarkerMessageDetector_0
    max_frequency: 0
    label: csapex::MarkerMessageDetector_0
    type: csapex::MarkerMessageDetector
    flipped: false
    exec_mode: 1
    logger_level: 1
    z: 725
    minimized: false
    pos:
      - 1225
      - 14
    color:
      - -1
      - -1
      - -1
  - max_frequency: 0
    label: csapex::TextDisplay_12
    pos:
      - -69
      - 391
    type: csapex::TextDisplay
    uuid: csapex::TextDisplay_12
    z: 728
    minimized: false
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        {}
    dict:
      - key: height
        int: 100
      - int: 153
        key: width
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    flipped: false
  - z: 730
    minimized: false
    dict:
      - key: height
        int: 100
      - key: width
        int: 153
    uuid: csapex::TextDisplay_13
    max_frequency: 0
    label: csapex::TextDisplay_12
    type: csapex::TextDisplay
    state:
      params:
        {}
      persistent_params:
        []
    pos:
      - -80
      - 170
    color:
      - -1
      - -1
      - -1
    flipped: false
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
  - exec_mode: 1
    logger_level: 1
    muted: false
    enabled: false
    flipped: false
    state:
      params:
        topic:
          type: value
          name: topic
          string: /pillars
        queue:
          max: 32
          step: 1
          int: 1
          min: 1
          type: range
          name: queue
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    max_frequency: 0
    label: csapex::ExportRos_5
    pos:
      - 225
      - 616
    type: csapex::ExportRos
    uuid: csapex::ExportRos_5
    z: 1075
    minimized: false
  - state:
      persistent_params:
        []
      params:
        to:
          type: value
          name: to
          string: /
        from:
          type: value
          name: from
          string: /
    pos:
      - 1936
      - 565
    color:
      - -1
      - -1
      - -1
    type: csapex::TransformPublisher
    uuid: csapex::TransformPublisher_1
    max_frequency: 0
    label: csapex::TransformPublisher_1
    flipped: false
    exec_mode: 1
    logger_level: 1
    minimized: false
    muted: false
    enabled: true
    z: 1066
  - state:
      persistent_params:
        []
      params:
        queue:
          type: range
          name: queue
          step: 1
          int: 1
          max: 32
          min: 1
        topic:
          string: /velodyne_points
          type: value
          name: topic
    exec_mode: 1
    logger_level: 1
    enabled: false
    flipped: false
    z: 1074
    muted: false
    minimized: false
    color:
      - -1
      - -1
      - -1
    max_frequency: 0
    label: csapex::ExportRos_6
    pos:
      - -55
      - 1460
    type: csapex::ExportRos
    uuid: csapex::ExportRos_6
  - state:
      params:
        {}
      persistent_params:
        []
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    flipped: false
    minimized: false
    z: 907
    color:
      - -1
      - -1
      - -1
    uuid: csapex::TransformToOdometry_1
    max_frequency: 0
    pos:
      - 1441
      - 1361
    label: csapex::TransformToOdometry_1
    type: csapex::TransformToOdometry
  - state:
      persistent_params:
        []
      params:
        {}
    flipped: false
    exec_mode: 1
    logger_level: 1
    minimized: false
    muted: false
    enabled: true
    z: 1064
    pos:
      - 1412
      - 782
    color:
      - -1
      - -1
      - -1
    type: csapex::TransformToOdometry
    uuid: csapex::TransformToOdometry_2
    max_frequency: 0
    label: csapex::TransformToOdometry_1
  - state:
      params:
        topic:
          type: value
          name: topic
          string: /odom/true/absolute
        queue:
          step: 1
          int: 1
          min: 1
          max: 32
          type: range
          name: queue
      persistent_params:
        []
    flipped: false
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    z: 1065
    minimized: false
    pos:
      - 2016
      - 796
    color:
      - -1
      - -1
      - -1
    uuid: csapex::ExportRos_7
    max_frequency: 0
    label: csapex::ExportRos_7
    type: csapex::ExportRos
  - state:
      persistent_params:
        []
      params:
        topic:
          type: value
          name: topic
          string: /odom/est
        queue:
          step: 1
          int: 1
          min: 1
          max: 32
          type: range
          name: queue
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    flipped: false
    z: 1072
    minimized: false
    color:
      - -1
      - -1
      - -1
    max_frequency: 0
    label: csapex::ExportRos_7
    pos:
      - 2106
      - 1299
    type: csapex::ExportRos
    uuid: csapex::ExportRos_8
  - state:
      persistent_params:
        []
      params:
        child_frame_id:
          type: value
          name: child_frame_id
          string: ""
        frame_id:
          type: value
          name: frame_id
          string: odom
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    flipped: false
    minimized: false
    color:
      - -1
      - -1
      - -1
    z: 1068
    max_frequency: 0
    label: csapex::ChangeTransformFrames_0
    pos:
      - 904
      - 1364
    type: csapex::ChangeTransformFrames
    uuid: csapex::ChangeTransformFrames_0
  - flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        queue:
          max: 32
          step: 1
          type: range
          name: queue
          int: 1
          min: 1
        topic:
          type: value
          name: topic
          string: /odom
    z: 922
    minimized: false
    pos:
      - -501
      - 1465
    color:
      - -1
      - -1
      - -1
    uuid: csapex::ExportRos_11
    max_frequency: 0
    label: csapex::ExportRos_6
    type: csapex::ExportRos
    muted: false
    enabled: true
  - flipped: false
    exec_mode: 1
    logger_level: 1
    type: csapex::ExportRos
    uuid: csapex::ExportRos_12
    max_frequency: 0
    label: csapex::ExportRos_7
    z: 1070
    minimized: false
    muted: false
    enabled: true
    pos:
      - 2056
      - 1024
    color:
      - -1
      - -1
      - -1
    state:
      params:
        queue:
          type: range
          max: 32
          step: 1
          name: queue
          int: 1
          min: 1
        topic:
          type: value
          name: topic
          string: /odom/true/relative
      persistent_params:
        []
  - state:
      persistent_params:
        []
      params:
        {}
    color:
      - -1
      - -1
      - -1
    z: 1071
    max_frequency: 0
    pos:
      - 1444
      - 941
    type: csapex::TransformToOdometry
    uuid: csapex::TransformToOdometry_5
    label: csapex::TransformToOdometry_1
    muted: false
    enabled: true
    flipped: false
    minimized: false
    logger_level: 1
    exec_mode: 1
  - type: csapex::TransformInverter
    uuid: csapex::TransformInverter_5
    minimized: false
    muted: false
    color:
      - -1
      - -1
      - -1
    z: 1067
    max_frequency: 0
    label: csapex::TransformInverter_5
    pos:
      - 1429
      - 629
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        {}
    exec_mode: 1
    logger_level: 1
threads:
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::FileImporter_0
    - id: 1
      uuid: csapex::PassThrough_0
    - id: 1
      uuid: csapex::PillarLocalization_0
    - uuid: csapex::ParamEstimator_0
      id: 1
    - uuid: csapex::PoseDifference_0
      id: 1
    - uuid: csapex::MarkerMessageDetector_0
      id: 1
    - uuid: csapex::EvaOptimizer_1
      id: 1
    - id: 1
      uuid: csapex::TextDisplay_12
    - id: 1
      uuid: csapex::TextDisplay_13
    - id: 1
      uuid: csapex::ExportRos_5
    - id: 1
      uuid: csapex::TransformPublisher_1
    - id: 1
      uuid: csapex::ExportRos_6
    - uuid: csapex::TransformToOdometry_1
      id: 1
    - uuid: csapex::TransformToOdometry_2
      id: 1
    - uuid: csapex::ExportRos_7
      id: 1
    - uuid: csapex::ExportRos_8
      id: 1
    - id: 1
      uuid: csapex::ChangeTransformFrames_0
    - uuid: csapex::ExportRos_11
      id: 1
    - id: 1
      uuid: csapex::ExportRos_12
    - id: 1
      uuid: csapex::TransformToOdometry_5
    - uuid: csapex::TransformInverter_5
      id: 1
  groups:
    - name: static
      id: 2