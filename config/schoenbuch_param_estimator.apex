#!/home/huskic/workspace/apex/devel/lib/csapex/csapex_main
nodes:
  - state:
      params:
        field:
          string: intensity
          type: set
          name: field
          txt: intensity
        interval:
          step: 0.01
          min: 0
          max: 1024
          type: interval
          name: interval
          double:
            - 160
            - 1024
        keep organized:
          type: value
          name: keep organized
          bool: true
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    z: 743
    minimized: false
    uuid: csapex::PassThrough_0
    max_frequency: 60
    label: csapex::PassThrough_0
    pos:
      - -49
      - 31
    muted: false
    enabled: true
    type: csapex::PassThrough
    flipped: false
    exec_mode: 1
    logger_level: 1
  - flipped: false
    exec_mode: 1
    color:
      - -1
      - -1
      - -1
    z: 716
    minimized: false
    muted: false
    enabled: true
    logger_level: 1
    uuid: csapex::PillarLocalization_0
    max_frequency: 60
    label: csapex::PillarLocalization_0
    pos:
      - 451
      - 362
    type: csapex::PillarLocalization
    state:
      params:
        cell/resolution:
          type: range
          name: cell/resolution
          double: 0.05
          min: 0.01
          max: 0.5
          step: 0.01
        clustering/max cluster diameter:
          type: range
          name: clustering/max cluster diameter
          double: 0.2
          min: 0
          max: 5
          step: 0.01
        clustering/max cluster size:
          type: range
          name: clustering/max cluster size
          int: 32
          min: 0
          max: 1024
          step: 1
        clustering/max distance euclidean:
          type: range
          name: clustering/max distance euclidean
          double: 0.5
          min: 0
          max: 1
          step: 0.01
        clustering/max distance on ring:
          type: range
          name: clustering/max distance on ring
          double: 0.05
          min: 0
          max: 0.1
          step: 0.001
        clustering/max distance vertically:
          type: range
          name: clustering/max distance vertically
          double: 0.05
          min: 0
          max: 0.1
          step: 0.001
        clustering/min cluster size:
          type: range
          name: clustering/min cluster size
          int: 32
          min: 0
          max: 1024
          step: 1
        distance/1:
          type: range
          name: distance/1
          double: 9.832000000000001
          min: 0.01
          max: 20
          step: 0.001
        distance/2:
          type: range
          name: distance/2
          double: 8.358000000000001
          min: 0.01
          max: 20
          step: 0.001
        distance/3:
          type: range
          name: distance/3
          double: 5.572
          min: 0.01
          max: 20
          step: 0.001
        min pts:
          type: range
          name: min pts
          int: 3
          min: 1
          max: 100
          step: 1
        pillar/min intensity:
          type: range
          name: pillar/min intensity
          double: 120
          min: 0
          max: 1024
          step: 0.1
        pillar/min points:
          type: range
          name: pillar/min points
          int: 32
          min: 0
          max: 512
          step: 1
        pillar/radius:
          type: range
          name: pillar/radius
          double: 0.055
          min: 0.001
          max: 1
          step: 0.001
        pillar/radius threshold:
          type: range
          name: pillar/radius threshold
          double: 0.055
          min: 0
          max: 1
          step: 0.001
        radius:
          type: range
          name: radius
          double: 0.055
          min: 0.01
          max: 1
          step: 0.001
        reset:
          type: trigger
          name: reset
        size:
          type: range
          name: size
          int: 28
          min: 1
          max: 50
          step: 1
        threshold:
          type: range
          name: threshold
          double: 0.2
          min: 0
          max: 1
          step: 0.001
      persistent_params:
        []
  - label: csapex::ParamEstimator_0
    max_frequency: 60
    uuid: csapex::ParamEstimator_0
    pos:
      - 204
      - 953
    color:
      - -1
      - -1
      - -1
    z: 722
    flipped: false
    minimized: false
    exec_mode: 1
    logger_level: 1
    muted: false
    enabled: true
    type: csapex::ParamEstimator
    state:
      params:
        alpha_l:
          type: range
          name: alpha_l
          interactive: true
          double: 0.753101183494327
          min: 0.5
          max: 1
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.9370531920207553
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
          double: -0.0899573295200678
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.5241745979489881
          min: 0.001
          max: 0.75
          step: 0.001
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.4501151731263338
          min: -0.75
          max: -0.001
          step: 0.001
      persistent_params:
        []
  - uuid: csapex::PoseDifference_0
    max_frequency: 60
    z: 715
    pos:
      - 545
      - 1184
    label: csapex::PoseDifference_0
    color:
      - -1
      - -1
      - -1
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
  - uuid: csapex::FileImporter_1
    type: csapex::FileImporter
    max_frequency: 60
    label: csapex::FileImporter_1
    pos:
      - -780
      - 1053
    color:
      - -1
      - -1
      - -1
    z: 745
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        bag/frame:
          type: range
          name: bag/frame
          int: 0
          min: 0
          max: 3839
          step: 1
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
          bool: false
        bag/publish tf:
          type: value
          name: bag/publish tf
          bool: false
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
          bool: false
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
        path:
          type: path
          name: path
          value: /home/huskic/Documents/Bagfiles/param_estimation/coarse_gravel_2016-03-07-16-34-32.bag
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
  - max_frequency: 60
    enabled: true
    uuid: csapex::EvaOptimizer_1
    label: csapex::EvaOptimizer_1
    pos:
      - -758
      - 179
    logger_level: 1
    muted: false
    color:
      - -1
      - -1
      - -1
    z: 742
    minimized: false
    flipped: false
    exec_mode: 1
    type: csapex::EvaOptimizer
    state:
      params:
        alpha_l:
          type: range
          name: alpha_l
          interactive: true
          double: 0.753101183494327
          min: 0.5
          max: 1
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.9370531920207553
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
          int: 60
          min: 60
          max: 60
          step: 1
        individuals/target:
          type: range
          name: individuals/target
          int: 30
          min: 30
          max: 30
          step: 1
        method:
          txt: Differential Evolution
          type: set
          name: method
          int: 1
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
          double: -0.0899573295200678
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.5241745979489881
          min: 0.001
          max: 0.75
          step: 0.001
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.4501151731263338
          min: -0.75
          max: -0.001
          step: 0.001
      persistent_params:
        - alpha_l
        - alpha_r
        - x_icr
        - y_icr_l
        - y_icr_r
  - uuid: csapex::TextDisplay_9
    flipped: false
    pos:
      - -289
      - 300
    max_frequency: 60
    minimized: false
    enabled: true
    muted: false
    color:
      - 255
      - 85
      - 0
    label: Last Fitness
    exec_mode: 1
    z: 698
    logger_level: 1
    type: csapex::TextDisplay
    dict:
      - key: height
        int: 100
      - key: width
        int: 100
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TextDisplay_10
    max_frequency: 60
    type: csapex::TextDisplay
    label: Best Fitness
    pos:
      - -284
      - 555
    color:
      - 255
      - 170
      - 0
    z: 699
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
        int: 100
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TimePlot_1
    max_frequency: 60
    type: csapex::TimePlot
    label: csapex::TimePlot_1
    pos:
      - 87
      - 1333
    color:
      - -1
      - -1
      - -1
    z: 644
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
        ~output/plot/number_of_points:
          type: value
          name: ~output/plot/number_of_points
          int: 1000
        ~output/time/relative:
          type: value
          name: ~output/time/relative
          bool: true
        ~output/time/seconds:
          type: value
          name: ~output/time/seconds
          bool: true
        ~plot/color/background:
          type: color
          name: ~plot/color/background
          values:
            - 255
            - 255
            - 255
        ~plot/color/fill:
          type: color
          name: ~plot/color/fill
          values:
            - 200
            - 200
            - 200
        ~plot/color/line:
          type: color
          name: ~plot/color/line
          values:
            - 100
            - 100
            - 255
        ~plot/height:
          type: range
          name: ~plot/height
          int: 320
          min: 128
          max: 4096
          step: 1
        ~plot/line/width:
          type: range
          name: ~plot/line/width
          double: 0
          min: 0
          max: 10
          step: 0.01
        ~plot/width:
          type: range
          name: ~plot/width
          int: 640
          min: 128
          max: 4096
          step: 1
      persistent_params:
        []
  - uuid: csapex::Relay_0
    color:
      - -1
      - -1
      - -1
    label: csapex::Relay_0
    max_frequency: 60
    pos:
      - -392
      - 1134
    z: 732
    minimized: false
    type: csapex::Relay
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
  - uuid: csapex::Relay_1
    type: csapex::Relay
    max_frequency: 60
    label: csapex::Relay_0
    pos:
      - -390
      - 1277
    color:
      - -1
      - -1
      - -1
    z: 744
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
  - uuid: csapex::Relay_2
    max_frequency: 60
    type: csapex::Relay
    label: csapex::Relay_0
    pos:
      - -377
      - 1424
    color:
      - -1
      - -1
      - -1
    z: 732
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
threads:
  groups: ~
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::PassThrough_0
    - uuid: csapex::PillarLocalization_0
      id: 1
    - uuid: csapex::ParamEstimator_0
      id: 1
    - id: 1
      uuid: csapex::PoseDifference_0
    - id: 1
      uuid: csapex::FileImporter_1
    - id: 1
      uuid: csapex::EvaOptimizer_1
    - id: 1
      uuid: csapex::TextDisplay_9
    - id: 1
      uuid: csapex::TextDisplay_10
    - uuid: csapex::TimePlot_1
      id: 1
    - uuid: csapex::Relay_0
      id: 1
    - uuid: csapex::Relay_1
      id: 1
    - uuid: csapex::Relay_2
      id: 1
uuid_map:
  csapex::SpinelloClustering: 1
  csapex::PassThrough: 1
  csapex::ParamEstimator: 1
  csapex::Relay: 3
  csapex::IntensityFilter: 1
  csapex::TextDisplay: 11
  csapex::StaticTransform: 1
  csapex::FileImporter: 2
  csapex::TransformPublisher: 1
  csapex::TransformInverter: 1
  csapex::ClusterPointcloud: 1
  csapex::PointCount: 1
  csapex::PillarLocalization: 1
  csapex::TransformCloud: 1
  csapex::LabelClusteredPointCloud: 2
  csapex::ColorPointCloud: 3
  csapex::TimePlot: 2
  csapex::ExportRos: 2
  csapex::PointCloudToIntensityImage: 1
  csapex::PointCloudToDepthImage: 1
  csapex::OutputDisplay: 2
  csapex::PoseDifference: 1
  csapex::Delay: 2
  csapex::CloudRenderer: 2
  csapex::EvaOptimizer: 2
  csapex::ImportRos: 1
adapters:
  []
connections:
  - targets:
      - csapex::PoseDifference_0:|:in_0
    uuid: csapex::ParamEstimator_0:|:out_0
    types:
      - default
  - uuid: csapex::Relay_0:|:out_0
    targets:
      - csapex::ParamEstimator_0:|:in_1
      - csapex::PillarLocalization_0:|:in_1
    types:
      - default
      - default
  - uuid: csapex::Relay_1:|:out_0
    targets:
      - csapex::PassThrough_0:|:in_0
    types:
      - default
  - uuid: csapex::Relay_2:|:out_0
    targets:
      - csapex::ParamEstimator_0:|:in_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_r
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_r
    types:
      - default
  - uuid: csapex::PassThrough_0:|:out_0
    targets:
      - csapex::PillarLocalization_0:|:in_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_0
    targets:
      - csapex::TextDisplay_9:|:in_0
    types:
      - default
  - uuid: csapex::PoseDifference_0:|:out_0
    targets:
      - csapex::EvaOptimizer_1:|:slot_4
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_y_icr_l
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_l
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_1
    targets:
      - csapex::TextDisplay_10:|:in_0
    types:
      - default
  - uuid: csapex::PillarLocalization_0:|:out_1
    targets:
      - csapex::PoseDifference_0:|:in_1
    types:
      - default
  - uuid: csapex::PillarLocalization_0:|:event_0
    targets:
      - csapex::ParamEstimator_0:|:slot_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:out_alpha_l
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_l
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_2
    targets:
      - csapex::FileImporter_1:|:slot_2
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_1
    targets:
      - csapex::FileImporter_1:|:slot_0
    types:
      - default
  - uuid: csapex::EvaOptimizer_1:|:event_4
    targets:
      - csapex::FileImporter_1:|:slot_0
      - csapex::PillarLocalization_0:|:slot_0
      - csapex::FileImporter_1:|:slot_1
    types:
      - default
      - default
      - default
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