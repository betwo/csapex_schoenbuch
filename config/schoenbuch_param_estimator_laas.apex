#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
adapters:
  []
connections:
  - uuid: csapex::EvaOptimizer_1:|:out_y_icr_r
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_r
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_4
    targets:
      - csapex::ParamEstimator_0:|:slot_0
      - csapex::PoseDifference_0:|:slot_0
      - csapex::GenerateRelativeTransform_0:|:slot_0
      - csapex::LAASImporter_1:|:slot_0
      - csapex::GenerateRelativeTransform_1:|:slot_0
    types:
      - default
      - default
      - default
      - default
      - default
  - uuid: csapex::LAASImporter_1:|:out_2
    targets:
      - csapex::ParamEstimator_0:|:in_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_l
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_l
    types:
      - default
  - uuid: csapex::LAASImporter_1:|:out_3
    targets:
      - csapex::ParamEstimator_0:|:in_1
      - csapex::ExportRos_13:|:in_0
      - csapex::MarkerMessageDetector_2:|:in_0
    types:
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
      - csapex::LAASImporter_1:|:slot_0
    types:
      - default
  - uuid: csapex::MarkerMessageDetector_0:|:event_0
    targets:
      - csapex::EvaOptimizer_1:|:slot_4
    types:
      - default
  - uuid: csapex::MarkerMessageDetector_2:|:event_0
    targets:
      - csapex::MakeScreenshot_0:|:slot_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_x_icr
    targets:
      - csapex::ParamEstimator_0:|:in_x_icr
    types:
      - default
  - uuid: csapex::GenericTextCombiner_0:|:out_0
    targets:
      - csapex::MakeScreenshot_0:|:in_format
    types:
      - default
  - uuid: csapex::GenerateRelativeTransform_1:|:out_0
    targets:
      - csapex::GenerateRelativeTransform_0:|:in_0
      - csapex::TransformToOdometry_2:|:in_0
      - csapex::TransformToOdometry_6:|:in_0
    types:
      - default
      - default
      - default
  - uuid: csapex::ParamEstimator_0:|:out_0
    targets:
      - csapex::PoseDifference_0:|:in_0
      - csapex::ChangeTransformFrames_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::TransformToOdometry_2:|:out_0
    targets:
      - csapex::ExportRos_7:|:in_0
    types:
      - default
  - uuid: csapex::TransformToOdometry_5:|:out_0
    targets:
      - csapex::ExportRos_12:|:in_0
    types:
      - default
  - uuid: csapex::TransformToOdometry_1:|:out_0
    targets:
      - csapex::ExportRos_8:|:in_0
    types:
      - default
  - uuid: csapex::ChangeTransformFrames_0:|:out_0
    targets:
      - csapex::TransformToOdometry_1:|:in_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_0
    targets:
      - csapex::TextDisplay_13:|:in_0
    types:
      - default
  - uuid: csapex::GenerateRelativeTransform_0:|:out_0
    targets:
      - csapex::PoseDifference_0:|:in_1
      - csapex::TransformToOdometry_5:|:in_0
    types:
      - default
      - default
  - uuid: csapex::TransformToOdometry_6:|:out_0
    targets:
      - csapex::ExportRos_11:|:in_0
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
      - csapex::GenericTextCombiner_0:|:in_0
    types:
      - default
      - default
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_r
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_r
    types:
      - default
  - uuid: csapex::LAASImporter_1:|:out_1
    targets:
      - csapex::GenerateRelativeTransform_1:|:in_0
    types:
      - default
threads:
  groups:
    - name: static
      id: 2
    - id: 3
      name: screenshot
    - name: export
      id: 4
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::ParamEstimator_0
    - uuid: csapex::PoseDifference_0
      id: 1
    - uuid: csapex::MarkerMessageDetector_0
      id: 1
    - id: 1
      uuid: csapex::EvaOptimizer_1
    - id: 3
      uuid: csapex::TextDisplay_12
    - id: 3
      uuid: csapex::TextDisplay_13
    - uuid: csapex::TransformToOdometry_1
      id: 4
    - id: 4
      uuid: csapex::TransformToOdometry_2
    - id: 4
      uuid: csapex::ExportRos_7
    - id: 4
      uuid: csapex::ExportRos_8
    - uuid: csapex::ChangeTransformFrames_0
      id: 4
    - uuid: csapex::ExportRos_11
      id: 4
    - id: 4
      uuid: csapex::ExportRos_12
    - uuid: csapex::TransformToOdometry_5
      id: 4
    - uuid: csapex::GenerateRelativeTransform_0
      id: 1
    - uuid: csapex::LAASImporter_1
      id: 1
    - id: 4
      uuid: csapex::TransformToOdometry_6
    - id: 4
      uuid: csapex::ExportRos_13
    - uuid: csapex::GenerateRelativeTransform_1
      id: 1
    - uuid: csapex::MakeScreenshot_0
      id: 3
    - uuid: csapex::MarkerMessageDetector_2
      id: 1
    - uuid: csapex::GenericTextCombiner_0
      id: 3
uuid_map:
  csapex::PointCloudToDepthImage: 1
  csapex::PointCloudToIntensityImage: 1
  csapex::PointCount: 1
  csapex::PoseDifference: 1
  csapex::OutputDisplay: 2
  csapex::ParamEstimator: 1
  csapex::PassThrough: 1
  csapex::PillarLocalization: 1
  csapex::SignTransformPublisher: 1
  csapex::SignalLight: 1
  csapex::SpinelloClustering: 1
  csapex::StaticTransform: 3
  csapex::TransformPublisher: 4
  csapex::TransformToOdometry: 7
  csapex::TransformToPose: 4
  csapex::LabelClusteredPointCloud: 2
  csapex::MakeScreenshot: 1
  csapex::MarkerMessageDetector: 3
  csapex::OdometryToTransform: 2
  csapex::GenericTextCombiner: 1
  csapex::ImportRos: 1
  csapex::IntensityFilter: 1
  csapex::LAASImporter: 2
  csapex::EvaOptimizer: 2
  csapex::ExportRos: 14
  csapex::ExtractMeasures: 2
  csapex::FileImporter: 1
  csapex::GenerateRelativeTransform: 2
  csapex::TextDisplay: 20
  csapex::TimePlot: 2
  csapex::TransformCloud: 1
  csapex::TransformInverter: 6
  csapex::CloudRenderer: 2
  csapex::ClusterPointcloud: 1
  csapex::Delay: 2
  csapex::ColorPointCloud: 3
  csapex::ChangeTransformFrames: 5
nodes:
  - z: 1189
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    type: csapex::ParamEstimator
    uuid: csapex::ParamEstimator_0
    max_frequency: 0
    label: csapex::ParamEstimator_0
    pos:
      - 349
      - 1262
    color:
      - -1
      - -1
      - -1
    state:
      params:
        alpha_l:
          type: range
          name: alpha_l
          interactive: true
          double: 1.053855441301181
          min: 0.5
          step: 0.001
          max: 1.1
        reset:
          type: trigger
          name: reset
        x_icr:
          type: range
          name: x_icr
          interactive: true
          double: -0.02554020091057027
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.3774708059054879
          min: 0.001
          max: 0.75
          step: 0.001
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.7134514914630731
          min: -0.75
          max: -0.001
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.8075196804556891
          min: 0.5
          max: 1.1
          step: 0.001
      persistent_params:
        []
  - type: csapex::PoseDifference
    uuid: csapex::PoseDifference_0
    max_frequency: 0
    label: csapex::PoseDifference_0
    pos:
      - 1116
      - 1144
    color:
      - -1
      - -1
      - -1
    z: 1224
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
  - type: csapex::EvaOptimizer
    uuid: csapex::EvaOptimizer_1
    max_frequency: 0
    label: csapex::EvaOptimizer_1
    pos:
      - -619
      - 41
    color:
      - -1
      - -1
      - -1
    z: 1259
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
          double: 1.053855441301181
          min: 0.5
          max: 1.1
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.8075196804556891
          min: 0.5
          max: 1.1
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
          int: 30
          min: -1
          max: 1024
          step: 1
        individuals/individuals:
          type: range
          name: individuals/individuals
          int: 200
          min: 10
          max: 200
          step: 1
        individuals/later_generations:
          type: range
          name: individuals/later_generations
          int: 20
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
          double: -0.02554020091057027
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.3774708059054879
          min: 0.001
          max: 0.75
          step: 0.001
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.7134514914630731
          min: -0.75
          max: -0.001
          step: 0.001
      persistent_params:
        - alpha_l
        - alpha_r
        - x_icr
        - y_icr_l
        - y_icr_r
  - max_frequency: 0
    uuid: csapex::MarkerMessageDetector_0
    logger_level: 1
    color:
      - -1
      - -1
      - -1
    label: csapex::MarkerMessageDetector_0
    flipped: false
    z: 725
    exec_mode: 1
    minimized: false
    pos:
      - 1225
      - 14
    muted: false
    enabled: true
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
    z: 1253
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
  - uuid: csapex::TransformToOdometry_1
    max_frequency: 0
    type: csapex::TransformToOdometry
    label: csapex::TransformToOdometry_1
    pos:
      - 1441
      - 1361
    color:
      - -1
      - -1
      - -1
    z: 1250
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
    z: 1265
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
          int: 32
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
    z: 1266
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
          int: 32
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
    z: 1251
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
      - -62
      - 1473
    color:
      - -1
      - -1
      - -1
    z: 1263
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
          int: 32
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
    z: 1264
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
          int: 32
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /odom/true/relative
      persistent_params:
        []
  - type: csapex::TransformToOdometry
    uuid: csapex::TransformToOdometry_5
    max_frequency: 0
    label: csapex::TransformToOdometry_1
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
  - type: csapex::GenerateRelativeTransform
    uuid: csapex::GenerateRelativeTransform_0
    max_frequency: 0
    label: csapex::GenerateRelativeTransform_0
    pos:
      - 343
      - 1039
    color:
      - -1
      - -1
      - -1
    z: 1153
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
  - state:
      persistent_params:
        []
      params:
        gps:
          type: path
          name: gps
          value: /home/buck/data/schoenbuch/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/GPS/GPS.log
        odometry:
          type: path
          name: odometry
          value: /home/buck/data/schoenbuch/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/pom/pom.log
        range:
          type: interval
          name: range
          int:
            - 2600
            - 4400
          min: 0
          max: 5446
          step: 1
        start:
          type: trigger
          name: start
        wheels:
          type: path
          name: wheels
          value: /home/buck/data/schoenbuch/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/rmp440/rmp440_internal.log
        frame:
          step: 1
          type: range
          name: frame
          int: 3660
          min: 0
          max: 5446
    type: csapex::LAASImporter
    uuid: csapex::LAASImporter_1
    max_frequency: 0
    label: csapex::LAASImporter_1
    pos:
      - -681
      - 1053
    color:
      - -1
      - -1
      - -1
    z: 1195
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
  - max_frequency: 0
    uuid: csapex::TransformToOdometry_6
    label: csapex::TransformToOdometry_6
    pos:
      - -474
      - 1398
    color:
      - -1
      - -1
      - -1
    type: csapex::TransformToOdometry
    z: 1252
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        {}
  - uuid: csapex::ExportRos_13
    type: csapex::ExportRos
    max_frequency: 0
    label: csapex::ExportRos_6
    pos:
      - -566
      - 1585
    color:
      - -1
      - -1
      - -1
    z: 1262
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        queue:
          type: range
          name: queue
          int: 32
          min: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /clock
  - type: csapex::GenerateRelativeTransform
    uuid: csapex::GenerateRelativeTransform_1
    max_frequency: 0
    label: csapex::GenerateRelativeTransform_0
    pos:
      - -226
      - 1138
    color:
      - -1
      - -1
      - -1
    z: 1197
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        {}
  - uuid: csapex::MakeScreenshot_0
    max_frequency: 0
    type: csapex::MakeScreenshot
    label: csapex::MakeScreenshot_0
    pos:
      - 464
      - 523
    color:
      - -1
      - -1
      - -1
    z: 1254
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        format:
          type: value
          name: format
          interactive: true
          string: laas_590574_%s_wxh.png
        path:
          type: path
          name: path
          value: /tmp/optimization
        quality:
          type: range
          name: quality
          int: 80
          min: 0
          max: 100
          step: 1
  - uuid: csapex::MarkerMessageDetector_2
    max_frequency: 0
    label: csapex::MarkerMessageDetector_0
    pos:
      - -51
      - 615
    type: csapex::MarkerMessageDetector
    color:
      - -1
      - -1
      - -1
    z: 1216
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        type:
          type: bitset
          name: type
          int: 6
  - uuid: csapex::GenericTextCombiner_0
    max_frequency: 0
    type: csapex::GenericTextCombiner
    label: csapex::GenericTextCombiner_0
    pos:
      - 449
      - 776
    color:
      - -1
      - -1
      - -1
    z: 1249
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        format:
          type: value
          name: format
          string: laas_$1_%s_$wx$h.png
        input count:
          type: value
          name: input count
          int: 1
        input names:
          type: string_list
          name: input names
          list:
            - accumulated error
            - accumulated error
        last_value:
          type: outtext
          name: last_value