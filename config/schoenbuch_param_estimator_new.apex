#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
uuid_map:
  csapex::Delay: 2
  csapex::TransformToOdometry: 3
  csapex::TransformToPose: 2
  csapex::PoseDifference: 1
  csapex::ClusterPointcloud: 1
  csapex::TransformCloud: 1
  csapex::PointCloudToDepthImage: 1
  csapex::SignTransformPublisher: 1
  csapex::TransformInverter: 2
  csapex::MarkerMessageDetector: 2
  csapex::ParamEstimator: 1
  csapex::SpinelloClustering: 1
  csapex::SignalLight: 1
  csapex::TransformPublisher: 2
  csapex::TextDisplay: 16
  csapex::StaticTransform: 2
  csapex::TimePlot: 2
  csapex::PointCount: 1
  csapex::PillarLocalization: 1
  csapex::OutputDisplay: 2
  csapex::CloudRenderer: 2
  csapex::ColorPointCloud: 3
  csapex::EvaOptimizer: 2
  csapex::OdometryToTransform: 1
  csapex::PassThrough: 1
  csapex::ImportRos: 1
  csapex::IntensityFilter: 1
  csapex::PointCloudToIntensityImage: 1
  csapex::LabelClusteredPointCloud: 2
  csapex::FileImporter: 1
  csapex::ExportRos: 9
threads:
  groups: ~
  assignments:
    - id: 1
      uuid: ~
    - uuid: csapex::FileImporter_0
      id: 1
    - uuid: csapex::PassThrough_0
      id: 1
    - uuid: csapex::PillarLocalization_0
      id: 1
    - uuid: csapex::ParamEstimator_0
      id: 1
    - id: 1
      uuid: csapex::PoseDifference_0
    - uuid: csapex::MarkerMessageDetector_0
      id: 1
    - uuid: csapex::EvaOptimizer_1
      id: 1
    - uuid: csapex::TextDisplay_12
      id: 1
    - uuid: csapex::TextDisplay_13
      id: 1
    - uuid: csapex::ExportRos_5
      id: 1
    - uuid: csapex::TransformPublisher_1
      id: 1
    - uuid: csapex::ExportRos_6
      id: 1
    - id: 1
      uuid: csapex::TransformToOdometry_1
    - uuid: csapex::TransformToOdometry_2
      id: 1
    - id: 1
      uuid: csapex::ExportRos_7
    - id: 1
      uuid: csapex::ExportRos_8
nodes:
  - label: csapex::FileImporter_0
    muted: false
    type: csapex::FileImporter
    minimized: false
    uuid: csapex::FileImporter_0
    flipped: false
    state:
      persistent_params:
        []
      params:
        playback/resend:
          type: value
          name: playback/resend
          bool: false
        directory/loop:
          name: directory/loop
          type: value
          bool: true
        playback/immediate:
          bool: true
          name: playback/immediate
          type: value
        directory/current:
          min: 0
          max: 0
          type: range
          step: 1
          int: 0
          name: directory/current
        import directory:
          name: import directory
          type: value
          bool: false
        directory/quit on end:
          type: value
          name: directory/quit on end
          bool: false
        directory/play:
          name: directory/play
          bool: true
          type: value
        bag/publish clock:
          name: bag/publish clock
          bool: true
          type: value
        path:
          name: path
          value: /home/buck/data/schoenbuch/coarse_gravel_2016-03-07-16-34-32.bag
          type: path
        directory/show parameters:
          bool: false
          type: value
          name: directory/show parameters
        bag/publish tf:
          bool: true
          type: value
          name: bag/publish tf
        directory:
          name: directory
          value: ""
          type: path
        playback/frequency:
          double: 23.5
          name: playback/frequency
          max: 256
          step: 0.5
          min: 1
          type: range
        bag/forward_tf:
          bool: true
          name: bag/forward_tf
          type: value
        directory/current_file:
          type: value
          name: directory/current_file
          string: ""
        cache:
          name: cache
          type: value
          bool: false
        directory/filter:
          type: value
          name: directory/filter
          string: .*
        directory/latch:
          name: directory/latch
          bool: false
          type: value
        bag/frame:
          int: 364
          type: range
          step: 1
          max: 3839
          min: 0
          name: bag/frame
        output_count:
          type: value
          name: output_count
          int: 4
        split_container_messages:
          bool: false
          type: value
          name: split_container_messages
        bag/play:
          bool: true
          type: value
          name: bag/play
        topic:
          type: set
          name: topic
          txt: /wheel_velocities
          string: /wheel_velocities
        bag/latch:
          bool: false
          type: value
          name: bag/latch
        recursive import:
          type: value
          bool: false
          name: recursive import
        bag/loop:
          bool: false
          type: value
          name: bag/loop
        directory/sort_numerically:
          name: directory/sort_numerically
          bool: false
          type: value
    z: 842
    logger_level: 1
    enabled: true
    pos:
      - -558
      - 885
    color:
      - -1
      - -1
      - -1
    exec_mode: 1
    max_frequency: 0
  - enabled: true
    flipped: false
    logger_level: 1
    max_frequency: 0
    color:
      - -1
      - -1
      - -1
    z: 817
    state:
      persistent_params:
        []
      params:
        interval:
          max: 1024
          type: interval
          name: interval
          double:
            - 148.21
            - 1024
          step: 0.01
          min: 0
        keep organized:
          name: keep organized
          bool: true
          type: value
        field:
          string: intensity
          txt: intensity
          name: field
          type: set
    pos:
      - 20
      - 814
    uuid: csapex::PassThrough_0
    muted: false
    type: csapex::PassThrough
    minimized: false
    exec_mode: 1
    label: csapex::PassThrough_0
  - muted: false
    color:
      - -1
      - -1
      - -1
    type: csapex::PillarLocalization
    z: 837
    label: csapex::PillarLocalization_0
    flipped: false
    minimized: false
    pos:
      - 650
      - 574
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        clustering/max cluster size:
          name: clustering/max cluster size
          int: 50
          min: 0
          step: 1
          type: range
          max: 1024
        pillar/radius:
          step: 0.001
          name: pillar/radius
          double: 0.065
          type: range
          min: 0.001
          max: 1
        clustering/max distance on ring:
          min: 0
          double: 0.043
          step: 0.001
          type: range
          name: clustering/max distance on ring
          max: 0.1
        clustering/max distance euclidean:
          type: range
          name: clustering/max distance euclidean
          double: 0.5
          min: 0
          max: 1
          step: 0.01
        clustering/max cluster diameter:
          min: 0
          step: 0.01
          type: range
          max: 5
          name: clustering/max cluster diameter
          double: 0.2
        pillar/min points:
          type: range
          name: pillar/min points
          int: 4
          step: 1
          max: 512
          min: 0
        clustering/min cluster size:
          step: 1
          type: range
          name: clustering/min cluster size
          int: 2
          min: 0
          max: 1024
        cell/resolution:
          name: cell/resolution
          type: range
          step: 0.01
          max: 0.5
          double: 0.05
          min: 0.01
        pillar/min intensity:
          step: 0.1
          max: 1024
          double: 140
          min: 0
          type: range
          name: pillar/min intensity
        clustering/max distance vertically:
          double: 0.026
          step: 0.001
          type: range
          name: clustering/max distance vertically
          min: 0
          max: 0.1
        min pts:
          step: 1
          max: 100
          min: 1
          type: range
          name: min pts
          int: 3
        distance/3:
          name: distance/3
          double: 5.572
          min: 0.01
          max: 20
          type: range
          step: 0.001
        distance/1:
          step: 0.001
          double: 9.832000000000001
          name: distance/1
          max: 20
          min: 0.01
          type: range
        undistortion:
          name: undistortion
          bool: false
          type: value
        distance/2:
          max: 20
          step: 0.001
          type: range
          name: distance/2
          double: 8.358000000000001
          min: 0.01
        threshold:
          type: range
          step: 0.001
          max: 1
          min: 0
          name: threshold
          double: 0.25
        size:
          step: 1
          max: 50
          int: 28
          min: 1
          type: range
          name: size
        reset:
          name: reset
          type: trigger
        radius:
          step: 0.001
          name: radius
          double: 0.055
          type: range
          max: 1
          min: 0.01
        pillar/radius threshold:
          name: pillar/radius threshold
          double: 0.067
          step: 0.001
          type: range
          max: 1
          min: 0
    uuid: csapex::PillarLocalization_0
    max_frequency: 0
    enabled: true
  - muted: false
    enabled: true
    max_frequency: 0
    label: csapex::ParamEstimator_0
    state:
      params:
        y_icr_l:
          double: 0.1647772620482741
          min: 0.001
          interactive: true
          type: range
          name: y_icr_l
          max: 0.75
          step: 0.001
        alpha_r:
          step: 0.001
          double: 0.9344573855458711
          type: range
          name: alpha_r
          interactive: true
          max: 1
          min: 0.5
        y_icr_r:
          name: y_icr_r
          interactive: true
          step: 0.001
          type: range
          double: -0.5034422033234451
          min: -0.75
          max: -0.001
        alpha_l:
          name: alpha_l
          interactive: true
          step: 0.001
          double: 0.5986515462573958
          min: 0.5
          max: 1
          type: range
        x_icr:
          step: 0.001
          type: range
          name: x_icr
          interactive: true
          min: -0.1
          max: 0
          double: -0.05237496236775577
        reset:
          name: reset
          type: trigger
      persistent_params:
        []
    flipped: false
    type: csapex::ParamEstimator
    uuid: csapex::ParamEstimator_0
    color:
      - -1
      - -1
      - -1
    pos:
      - 347
      - 1217
    exec_mode: 1
    logger_level: 1
    minimized: false
    z: 691
  - uuid: csapex::PoseDifference_0
    max_frequency: 0
    state:
      params:
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    label: csapex::PoseDifference_0
    pos:
      - 1116
      - 1144
    type: csapex::PoseDifference
    exec_mode: 1
    logger_level: 1
    minimized: false
    z: 726
    muted: false
    enabled: true
    flipped: false
  - type: csapex::EvaOptimizer
    pos:
      - -619
      - 41
    label: csapex::EvaOptimizer_1
    uuid: csapex::EvaOptimizer_1
    max_frequency: 0
    z: 865
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        - alpha_l
        - alpha_r
        - x_icr
        - y_icr_l
        - y_icr_r
      params:
        start:
          name: start
          type: trigger
        stop:
          type: trigger
          name: stop
        generations:
          type: range
          name: generations
          max: 1024
          step: 1
          int: 2
          min: -1
        alpha_r:
          type: range
          name: alpha_r
          double: 0.9344573855458711
          min: 0.5
          interactive: true
          max: 1
          step: 0.001
        server port:
          string: 2342
          type: value
          name: server port
        server name:
          type: value
          name: server name
          string: localhost
        finish_immediately:
          bool: false
          type: value
          name: finish_immediately
        set best:
          type: trigger
          name: set best
        population:
          type: progress
          name: population
        perform_evaluation:
          name: perform_evaluation
          bool: false
          type: value
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          step: 0.001
          max: 0.75
          double: 0.1647772620482741
          min: 0.001
        finish:
          name: finish
          type: trigger
        y_icr_r:
          name: y_icr_r
          interactive: true
          min: -0.75
          double: -0.5034422033234451
          type: range
          step: 0.001
          max: -0.001
        alpha_l:
          name: alpha_l
          interactive: true
          double: 0.5986515462573958
          min: 0.5
          type: range
          max: 1
          step: 0.001
        individuals/individuals:
          max: 60
          step: 1
          type: range
          name: individuals/individuals
          int: 60
          min: 60
        x_icr:
          max: 0
          min: -0.1
          interactive: true
          double: -0.05237496236775577
          step: 0.001
          type: range
          name: x_icr
        method:
          txt: Differential Evolution
          int: 1
          type: set
          name: method
        individuals/target:
          min: 30
          max: 30
          step: 1
          int: 30
          type: range
          name: individuals/target
        generation:
          type: progress
          name: generation
    minimized: false
    exec_mode: 1
    logger_level: 1
    enabled: true
    flipped: false
    muted: false
  - type: csapex::MarkerMessageDetector
    uuid: csapex::MarkerMessageDetector_0
    max_frequency: 0
    label: csapex::MarkerMessageDetector_0
    minimized: false
    muted: false
    enabled: true
    color:
      - -1
      - -1
      - -1
    z: 725
    logger_level: 1
    flipped: false
    exec_mode: 1
    pos:
      - 1225
      - 14
    state:
      params:
        type:
          type: bitset
          name: type
          int: 1
      persistent_params:
        []
  - state:
      persistent_params:
        []
      params:
        {}
    minimized: false
    muted: false
    max_frequency: 0
    label: csapex::TextDisplay_12
    dict:
      - key: height
        int: 100
      - int: 153
        key: width
    z: 728
    pos:
      - -69
      - 391
    color:
      - -1
      - -1
      - -1
    type: csapex::TextDisplay
    uuid: csapex::TextDisplay_12
    exec_mode: 1
    logger_level: 1
    enabled: true
    flipped: false
  - color:
      - -1
      - -1
      - -1
    z: 730
    minimized: false
    flipped: false
    pos:
      - -80
      - 170
    label: csapex::TextDisplay_12
    type: csapex::TextDisplay
    uuid: csapex::TextDisplay_13
    max_frequency: 0
    dict:
      - key: height
        int: 100
      - key: width
        int: 153
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
    muted: false
    enabled: true
  - color:
      - -1
      - -1
      - -1
    pos:
      - 225
      - 616
    uuid: csapex::ExportRos_5
    max_frequency: 0
    label: csapex::ExportRos_5
    type: csapex::ExportRos
    state:
      params:
        topic:
          name: topic
          string: /pillars
          type: value
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          step: 1
          max: 32
      persistent_params:
        []
    flipped: false
    minimized: false
    z: 759
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
  - minimized: false
    muted: false
    label: csapex::TransformPublisher_1
    type: csapex::TransformPublisher
    uuid: csapex::TransformPublisher_1
    max_frequency: 0
    enabled: true
    flipped: false
    pos:
      - 1608
      - 712
    color:
      - -1
      - -1
      - -1
    z: 841
    state:
      params:
        to:
          type: value
          name: to
          string: /pillars
        from:
          string: /base_link
          type: value
          name: from
      persistent_params:
        []
    exec_mode: 1
    logger_level: 1
  - type: csapex::ExportRos
    z: 807
    pos:
      - -55
      - 1460
    state:
      params:
        queue:
          type: range
          max: 32
          min: 1
          step: 1
          name: queue
          int: 1
        topic:
          string: /velodyne_points
          type: value
          name: topic
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    minimized: false
    muted: false
    uuid: csapex::ExportRos_6
    max_frequency: 0
    label: csapex::ExportRos_6
  - state:
      persistent_params:
        []
      params:
        {}
    exec_mode: 1
    logger_level: 1
    uuid: csapex::TransformToOdometry_1
    max_frequency: 60
    label: csapex::TransformToOdometry_1
    minimized: false
    muted: false
    enabled: true
    flipped: false
    z: 843
    color:
      - -1
      - -1
      - -1
    pos:
      - 1441
      - 1361
    type: csapex::TransformToOdometry
  - flipped: false
    exec_mode: 1
    max_frequency: 60
    pos:
      - 1510
      - 1593
    label: csapex::TransformToOdometry_1
    logger_level: 1
    color:
      - -1
      - -1
      - -1
    state:
      params:
        {}
      persistent_params:
        []
    type: csapex::TransformToOdometry
    uuid: csapex::TransformToOdometry_2
    muted: false
    enabled: true
    minimized: false
    z: 866
  - enabled: true
    flipped: false
    z: 848
    minimized: false
    muted: false
    state:
      params:
        topic:
          type: value
          name: topic
          string: /odom/true
        queue:
          name: queue
          int: 1
          max: 32
          min: 1
          step: 1
          type: range
      persistent_params:
        []
    type: csapex::ExportRos
    uuid: csapex::ExportRos_7
    max_frequency: 60
    label: csapex::ExportRos_7
    exec_mode: 1
    logger_level: 1
    pos:
      - 2077
      - 1217
    color:
      - -1
      - -1
      - -1
  - max_frequency: 60
    label: csapex::ExportRos_7
    pos:
      - 2129
      - 1423
    color:
      - -1
      - -1
      - -1
    z: 851
    muted: false
    enabled: true
    logger_level: 1
    flipped: false
    exec_mode: 1
    minimized: false
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          step: 1
          min: 1
          max: 32
        topic:
          type: value
          name: topic
          string: /odom/est
      persistent_params:
        []
    type: csapex::ExportRos
    uuid: csapex::ExportRos_8
adapters:
  []
connections:
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_l
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_l
    types:
      - default
  - uuid: csapex::FileImporter_0:|:out_2
    targets:
      - csapex::PassThrough_0:|:in_0
      - csapex::ExportRos_6:|:in_0
    types:
      - default
      - default
  - targets:
      - csapex::ParamEstimator_0:|:in_alpha_r
    types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_alpha_r
  - types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_0
    targets:
      - csapex::TextDisplay_13:|:in_0
  - uuid: csapex::FileImporter_0:|:out_0
    targets:
      - csapex::ParamEstimator_0:|:in_1
      - csapex::PillarLocalization_0:|:in_1
    types:
      - default
      - default
  - targets:
      - csapex::PoseDifference_0:|:in_0
      - csapex::TransformToOdometry_1:|:in_0
    types:
      - default
      - default
    uuid: csapex::ParamEstimator_0:|:out_0
  - types:
      - default
      - default
    uuid: csapex::PassThrough_0:|:out_0
    targets:
      - csapex::PillarLocalization_0:|:in_0
      - csapex::ExportRos_5:|:in_0
  - targets:
      - csapex::TransformPublisher_1:|:in_0
    types:
      - default
    uuid: csapex::PillarLocalization_0:|:out_0
  - targets:
      - csapex::ExportRos_8:|:in_0
    types:
      - default
    uuid: csapex::TransformToOdometry_1:|:out_0
  - uuid: csapex::TransformToOdometry_2:|:out_0
    targets:
      - csapex::ExportRos_7:|:in_0
    types:
      - default
  - targets:
      - csapex::FileImporter_0:|:slot_2
    types:
      - default
    uuid: csapex::EvaOptimizer_1:|:event_2
  - uuid: csapex::EvaOptimizer_1:|:event_4
    targets:
      - csapex::FileImporter_0:|:slot_0
      - csapex::PillarLocalization_0:|:slot_0
      - csapex::ParamEstimator_0:|:slot_0
    types:
      - default
      - default
      - default
  - targets:
      - csapex::ParamEstimator_0:|:in_y_icr_r
    types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_y_icr_r
  - types:
      - default
    targets:
      - csapex::ParamEstimator_0:|:in_x_icr
    uuid: csapex::EvaOptimizer_1:|:out_x_icr
  - targets:
      - csapex::EvaOptimizer_1:|:slot_4
    uuid: csapex::MarkerMessageDetector_0:|:event_0
    types:
      - default
  - targets:
      - csapex::FileImporter_0:|:slot_1
    uuid: csapex::EvaOptimizer_1:|:event_0
    types:
      - default
  - types:
      - default
    uuid: csapex::EvaOptimizer_1:|:out_1
    targets:
      - csapex::TextDisplay_12:|:in_0
  - targets:
      - csapex::PoseDifference_0:|:in_1
      - csapex::TransformToOdometry_2:|:in_0
    types:
      - default
      - default
    uuid: csapex::PillarLocalization_0:|:out_1
  - types:
      - default
      - default
    uuid: csapex::PoseDifference_0:|:out_1
    targets:
      - csapex::EvaOptimizer_1:|:slot_4
      - csapex::MarkerMessageDetector_0:|:in_0
  - uuid: csapex::EvaOptimizer_1:|:out_y_icr_l
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_l
    types:
      - default
  - types:
      - default
    uuid: csapex::FileImporter_0:|:out_3
    targets:
      - csapex::ParamEstimator_0:|:in_0