#!/home/huskic/workspace/apex/devel/lib/csapex/csapex_main
threads:
  groups:
    - id: 2
      name: static
  assignments:
    - id: 1
      uuid: ~
    - uuid: csapex::FileImporter_0
      id: 1
    - id: 1
      uuid: csapex::ParamEstimator_0
    - uuid: csapex::PoseDifference_0
      id: 1
    - uuid: csapex::MarkerMessageDetector_0
      id: 1
    - id: 1
      uuid: csapex::EvaOptimizer_1
    - id: 1
      uuid: csapex::TextDisplay_12
    - id: 1
      uuid: csapex::TextDisplay_13
    - uuid: csapex::TransformPublisher_1
      id: 1
    - uuid: csapex::ExportRos_6
      id: 1
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
    - uuid: csapex::TransformToOdometry_5
      id: 1
    - uuid: csapex::TransformInverter_5
      id: 1
    - uuid: csapex::OdometryToTransform_1
      id: 1
    - uuid: csapex::GenerateRelativeTransform_0
      id: 1
connections:
  - uuid: csapex::FileImporter_0:|:out_3
    targets:
      - csapex::ParamEstimator_0:|:in_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_l
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_l
    types:
      - default
  - uuid: csapex::FileImporter_0:|:out_2
    targets:
      - csapex::ExportRos_6:|:in_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_2
    targets:
      - csapex::FileImporter_0:|:slot_2
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_4
    targets:
      - csapex::FileImporter_0:|:slot_0
      - csapex::ParamEstimator_0:|:slot_0
      - csapex::PoseDifference_0:|:slot_0
      - csapex::GenerateRelativeTransform_0:|:slot_0
    types:
      - default
      - default
      - default
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_y_icr_l
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_l
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_0
    targets:
      - csapex::FileImporter_0:|:slot_1
    types:
      - default
  - uuid: csapex::MarkerMessageDetector_0:|:event_0
    targets:
      - csapex::EvaOptimizer_1:|:slot_4
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_x_icr
    targets:
      - csapex::ParamEstimator_0:|:in_x_icr
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_y_icr_r
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_r
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_1
    targets:
      - csapex::TextDisplay_12:|:in_0
    types:
      - default
  - uuid: csapex::PoseDifference_0:|:out_1
    targets:
      - csapex::EvaOptimizer_1:|:slot_4
      - csapex::MarkerMessageDetector_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_r
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_r
    types:
      - default
  - uuid: csapex::OdometryToTransform_1:|:out_0
    targets:
      - csapex::GenerateRelativeTransform_0:|:in_0
      - csapex::TransformToOdometry_2:|:in_0
    types:
      - default
      - default
  - uuid: csapex::GenerateRelativeTransform_0:|:out_0
    targets:
      - csapex::PoseDifference_0:|:in_1
      - csapex::TransformToOdometry_5:|:in_0
    types:
      - default
      - default
  - uuid: csapex::FileImporter_0:|:out_0
    targets:
      - csapex::ParamEstimator_0:|:in_1
      - csapex::ExportRos_11:|:in_0
      - csapex::OdometryToTransform_1:|:in_0
    types:
      - default
      - default
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_0
    targets:
      - csapex::TextDisplay_13:|:in_0
    types:
      - default
  - uuid: csapex::ChangeTransformFrames_0:|:out_0
    targets:
      - csapex::TransformToOdometry_1:|:in_0
    types:
      - default
  - uuid: csapex::TransformInverter_5:|:out_0
    targets:
      - csapex::TransformPublisher_1:|:in_0
    types:
      - default
  - uuid: csapex::TransformToOdometry_1:|:out_0
    targets:
      - csapex::ExportRos_8:|:in_0
    types:
      - default
  - uuid: csapex::TransformToOdometry_5:|:out_0
    targets:
      - csapex::ExportRos_12:|:in_0
    types:
      - default
  - uuid: csapex::TransformToOdometry_2:|:out_0
    targets:
      - csapex::ExportRos_7:|:in_0
    types:
      - default
  - uuid: csapex::ParamEstimator_0:|:out_0
    targets:
      - csapex::PoseDifference_0:|:in_0
      - csapex::ChangeTransformFrames_0:|:in_0
    types:
      - default
      - default
uuid_map:
  csapex::ExtractMeasures: 2
  csapex::FileImporter: 1
  csapex::GenerateRelativeTransform: 1
  csapex::ImportRos: 1
  csapex::IntensityFilter: 1
  csapex::LabelClusteredPointCloud: 2
  csapex::MarkerMessageDetector: 2
  csapex::SignTransformPublisher: 1
  csapex::SignalLight: 1
  csapex::SpinelloClustering: 1
  csapex::StaticTransform: 3
  csapex::TextDisplay: 19
  csapex::TimePlot: 2
  csapex::TransformCloud: 1
  csapex::TransformInverter: 6
  csapex::TransformPublisher: 4
  csapex::TransformToOdometry: 6
  csapex::TransformToPose: 4
  csapex::OdometryToTransform: 2
  csapex::OutputDisplay: 2
  csapex::ParamEstimator: 1
  csapex::PassThrough: 1
  csapex::PillarLocalization: 1
  csapex::PointCloudToDepthImage: 1
  csapex::PointCloudToIntensityImage: 1
  csapex::PointCount: 1
  csapex::PoseDifference: 1
  csapex::ChangeTransformFrames: 5
  csapex::CloudRenderer: 2
  csapex::ClusterPointcloud: 1
  csapex::ColorPointCloud: 3
  csapex::Delay: 2
  csapex::EvaOptimizer: 2
  csapex::ExportRos: 13
nodes:
  - minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    type: csapex::FileImporter
    uuid: csapex::FileImporter_0
    max_frequency: 0
    label: csapex::FileImporter_0
    pos:
      - -558
      - 885
    color:
      - -1
      - -1
      - -1
    z: 1116
    state:
      params:
        bag/latch:
          type: value
          name: bag/latch
          bool: false
        bag/loop:
          type: value
          name: bag/loop
          bool: false
        bag/play:
          type: value
          name: bag/play
          bool: true
        bag/publish clock:
          type: value
          name: bag/publish clock
          bool: true
        bag/publish tf:
          type: value
          name: bag/publish tf
          bool: true
        cache:
          type: value
          name: cache
          bool: false
        directory:
          type: path
          name: directory
          value: ""
        directory/current:
          type: range
          name: directory/current
          int: 0
          min: 0
          max: 0
          step: 1
        directory/current_file:
          type: value
          name: directory/current_file
          string: ""
        directory/filter:
          type: value
          name: directory/filter
          string: .*
        directory/latch:
          type: value
          name: directory/latch
          bool: false
        directory/loop:
          type: value
          name: directory/loop
          bool: true
        directory/play:
          type: value
          name: directory/play
          bool: true
        directory/quit on end:
          type: value
          name: directory/quit on end
          bool: false
        directory/show parameters:
          type: value
          name: directory/show parameters
          bool: false
        directory/sort_numerically:
          type: value
          name: directory/sort_numerically
          bool: false
        import directory:
          type: value
          name: import directory
          bool: false
        bag/forward_tf:
          type: value
          name: bag/forward_tf
          bool: true
        bag/frame:
          type: range
          name: bag/frame
          int: 24
          min: 0
          step: 1
          max: 3839
        output_count:
          type: value
          name: output_count
          int: 4
        path:
          type: path
          name: path
          value: /home/huskic/Documents/Bagfiles/param_estimation/coarse_gravel_2016-03-07-16-34-32.bag
        playback/frequency:
          type: range
          name: playback/frequency
          double: 23.5
          min: 1
          max: 256
          step: 0.5
        playback/immediate:
          type: value
          name: playback/immediate
          bool: true
        playback/resend:
          type: value
          name: playback/resend
          bool: false
        recursive import:
          type: value
          name: recursive import
          bool: false
        split_container_messages:
          type: value
          name: split_container_messages
          bool: false
        topic:
          type: set
          name: topic
          txt: /wheel_velocities
          string: /wheel_velocities
      persistent_params:
        []
  - max_frequency: 0
    label: csapex::ParamEstimator_0
    pos:
      - 349
      - 1262
    muted: false
    uuid: csapex::ParamEstimator_0
    enabled: true
    logger_level: 1
    color:
      - -1
      - -1
      - -1
    flipped: false
    z: 1108
    minimized: false
    exec_mode: 1
    type: csapex::ParamEstimator
    state:
      params:
        alpha_l:
          type: range
          name: alpha_l
          interactive: true
          double: 0.939898571067304
          min: 0.5
          max: 1
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.9447218960603014
          min: 0.5
          max: 1
          step: 0.001
        reset:
          type: trigger
          name: reset
        x_icr:
          type: range
          name: x_icr
          interactive: true
          double: -0.03630529928427366
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.3029604849159783
          min: 0.001
          max: 0.75
          step: 0.001
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.1845782579198468
          min: -0.75
          max: -0.001
          step: 0.001
      persistent_params:
        []
  - uuid: csapex::PoseDifference_0
    z: 1090
    pos:
      - 1116
      - 1144
    max_frequency: 0
    color:
      - -1
      - -1
      - -1
    label: csapex::PoseDifference_0
    type: csapex::PoseDifference
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
  - uuid: csapex::EvaOptimizer_1
    type: csapex::EvaOptimizer
    max_frequency: 0
    label: csapex::EvaOptimizer_1
    pos:
      - -619
      - 41
    color:
      - -1
      - -1
      - -1
    z: 1118
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        alpha_l:
          type: range
          name: alpha_l
          interactive: true
          double: 0.939898571067304
          min: 0.5
          max: 1
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.9447218960603014
          min: 0.5
          max: 1
          step: 0.001
        finish:
          type: trigger
          name: finish
        finish_immediately:
          type: value
          name: finish_immediately
          bool: false
        fitness level:
          type: progress
          name: fitness level
        generation:
          type: progress
          name: generation
        generations:
          type: range
          name: generations
          int: -1
          min: -1
          max: 1024
          step: 1
        individuals/individuals:
          type: range
          name: individuals/individuals
          int: 100
          min: 10
          max: 100
          step: 1
        individuals/later_generations:
          type: range
          name: individuals/later_generations
          int: 15
          min: 10
          max: 30
          step: 1
        individuals/target:
          type: range
          name: individuals/target
          int: 30
          min: 30
          max: 30
          step: 1
        method:
          type: set
          name: method
          txt: Differential Evolution
          int: 1
        perform_evaluation:
          type: value
          name: perform_evaluation
          bool: true
        population:
          type: progress
          name: population
        server name:
          type: value
          name: server name
          string: localhost
        server port:
          type: value
          name: server port
          string: 2342
        set best:
          type: trigger
          name: set best
        start:
          type: trigger
          name: start
        stop:
          type: trigger
          name: stop
        x_icr:
          type: range
          name: x_icr
          interactive: true
          double: -0.03630529928427366
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.3029604849159783
          min: 0.001
          max: 0.75
          step: 0.001
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.1845782579198468
          min: -0.75
          max: -0.001
          step: 0.001
      persistent_params:
        - alpha_l
        - alpha_r
        - x_icr
        - y_icr_l
        - y_icr_r
  - uuid: csapex::MarkerMessageDetector_0
    color:
      - -1
      - -1
      - -1
    max_frequency: 0
    flipped: false
    exec_mode: 1
    enabled: true
    z: 725
    minimized: false
    label: csapex::MarkerMessageDetector_0
    pos:
      - 1225
      - 14
    muted: false
    logger_level: 1
    type: csapex::MarkerMessageDetector
    state:
      params:
        type:
          type: bitset
          name: type
          int: 1
      persistent_params:
        []
  - uuid: csapex::TextDisplay_12
    max_frequency: 0
    type: csapex::TextDisplay
    label: csapex::TextDisplay_12
    pos:
      - -69
      - 391
    color:
      - -1
      - -1
      - -1
    z: 728
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    dict:
      - key: height
        int: 100
      - key: width
        int: 153
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TextDisplay_13
    max_frequency: 0
    type: csapex::TextDisplay
    label: csapex::TextDisplay_12
    pos:
      - -80
      - 170
    color:
      - -1
      - -1
      - -1
    z: 730
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    dict:
      - key: height
        int: 100
      - key: width
        int: 153
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TransformPublisher_1
    max_frequency: 0
    type: csapex::TransformPublisher
    label: csapex::TransformPublisher_1
    pos:
      - 1936
      - 565
    color:
      - -1
      - -1
      - -1
    z: 1066
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        from:
          type: value
          name: from
          string: /
        to:
          type: value
          name: to
          string: /
      persistent_params:
        []
  - uuid: csapex::ExportRos_6
    max_frequency: 0
    label: csapex::ExportRos_6
    type: csapex::ExportRos
    pos:
      - -55
      - 1460
    color:
      - -1
      - -1
      - -1
    z: 1099
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /velodyne_points
      persistent_params:
        []
  - uuid: csapex::TransformToOdometry_1
    max_frequency: 0
    label: csapex::TransformToOdometry_1
    type: csapex::TransformToOdometry
    pos:
      - 1441
      - 1361
    color:
      - -1
      - -1
      - -1
    z: 907
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TransformToOdometry_2
    type: csapex::TransformToOdometry
    max_frequency: 0
    label: csapex::TransformToOdometry_1
    pos:
      - 1412
      - 782
    color:
      - -1
      - -1
      - -1
    z: 1121
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::ExportRos_7
    type: csapex::ExportRos
    max_frequency: 0
    label: csapex::ExportRos_7
    pos:
      - 2016
      - 796
    color:
      - -1
      - -1
      - -1
    z: 1065
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /odom/true/absolute
      persistent_params:
        []
  - max_frequency: 0
    uuid: csapex::ExportRos_8
    label: csapex::ExportRos_7
    type: csapex::ExportRos
    pos:
      - 2106
      - 1299
    color:
      - -1
      - -1
      - -1
    z: 1072
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /odom/est
      persistent_params:
        []
  - uuid: csapex::ChangeTransformFrames_0
    max_frequency: 0
    label: csapex::ChangeTransformFrames_0
    type: csapex::ChangeTransformFrames
    pos:
      - 904
      - 1364
    color:
      - -1
      - -1
      - -1
    z: 1068
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        child_frame_id:
          type: value
          name: child_frame_id
          string: ""
        frame_id:
          type: value
          name: frame_id
          string: odom
      persistent_params:
        []
  - uuid: csapex::ExportRos_11
    max_frequency: 0
    label: csapex::ExportRos_6
    type: csapex::ExportRos
    pos:
      - -501
      - 1465
    color:
      - -1
      - -1
      - -1
    z: 922
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /odom
      persistent_params:
        []
  - uuid: csapex::ExportRos_12
    max_frequency: 0
    label: csapex::ExportRos_7
    type: csapex::ExportRos
    pos:
      - 2056
      - 1024
    color:
      - -1
      - -1
      - -1
    z: 1070
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /odom/true/relative
      persistent_params:
        []
  - max_frequency: 0
    uuid: csapex::TransformToOdometry_5
    label: csapex::TransformToOdometry_1
    type: csapex::TransformToOdometry
    pos:
      - 1444
      - 941
    color:
      - -1
      - -1
      - -1
    z: 1071
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TransformInverter_5
    max_frequency: 0
    type: csapex::TransformInverter
    label: csapex::TransformInverter_5
    pos:
      - 1429
      - 629
    color:
      - -1
      - -1
      - -1
    z: 1067
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::OdometryToTransform_1
    type: csapex::OdometryToTransform
    max_frequency: 60
    label: csapex::OdometryToTransform_1
    pos:
      - -76
      - 833
    color:
      - -1
      - -1
      - -1
    z: 1117
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::GenerateRelativeTransform
    uuid: csapex::GenerateRelativeTransform_0
    max_frequency: 60
    label: csapex::GenerateRelativeTransform_0
    pos:
      - 427
      - 925
    color:
      - -1
      - -1
      - -1
    z: 1122
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        {}
      persistent_params:
        []
adapters:
  []