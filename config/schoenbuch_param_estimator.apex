#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - id: 0
      uuid: csapex::FileImporter_0
    - id: 1
      uuid: csapex::ClusterPointcloud_0
    - uuid: csapex::PassThrough_0
      id: 1
    - uuid: csapex::PillarLocalization_0
      id: 1
    - id: 1
      uuid: csapex::ParamEstimator_0
    - uuid: csapex::PoseDifference_0
      id: 1
    - uuid: csapex::EvaOptimizer_0
      id: 1
    - uuid: csapex::Delay_1
      id: 1
    - uuid: csapex::TimePlot_1
      id: 1
uuid_map:
  csapex::CloudRenderer: 2
  csapex::ClusterPointcloud: 1
  csapex::EvaOptimizer: 1
  csapex::LabelClusteredPointCloud: 2
  csapex::OutputDisplay: 2
  csapex::PointCloudToDepthImage: 1
  csapex::StaticTransform: 1
  csapex::PointCloudToIntensityImage: 1
  csapex::ColorPointCloud: 3
  csapex::TextDisplay: 9
  csapex::FileImporter: 1
  csapex::ImportRos: 1
  csapex::SpinelloClustering: 1
  csapex::TransformCloud: 1
  csapex::PointCount: 1
  csapex::IntensityFilter: 1
  csapex::Delay: 2
  csapex::PoseDifference: 1
  csapex::PassThrough: 1
  csapex::PillarLocalization: 1
  csapex::ExportRos: 2
  csapex::TimePlot: 2
  csapex::ParamEstimator: 1
  csapex::TransformPublisher: 1
  csapex::TransformInverter: 1
nodes:
  - minimized: false
    label: csapex::FileImporter_0
    pos:
      - -495
      - 911
    flipped: false
    enabled: true
    state:
      persistent_params:
        []
      params:
        bag/latch:
          type: value
          name: bag/latch
          bool: false
        bag/publish tf:
          name: bag/publish tf
          bool: false
          type: value
        cache:
          type: value
          name: cache
          bool: false
        directory/loop:
          type: value
          name: directory/loop
          bool: true
        recursive import:
          type: value
          name: recursive import
          bool: false
        directory:
          type: path
          name: directory
          value: ""
        bag/frame:
          min: 0
          name: bag/frame
          int: 718
          type: range
          max: 7222
          step: 1
        bag/publish clock:
          bool: false
          type: value
          name: bag/publish clock
        playback/immediate:
          type: value
          name: playback/immediate
          bool: true
        playback/frequency:
          double: 23.5
          min: 1
          max: 256
          step: 0.5
          type: range
          name: playback/frequency
        path:
          value: /home/buck/data/schoenbuch/param_estimation/coarse_gravel_2016-02-18-16-34-32.bag
          type: path
          name: path
        bag/play:
          type: value
          name: bag/play
          bool: true
        import directory:
          bool: false
          type: value
          name: import directory
        topic:
          type: set
          name: topic
          txt: /wheel_velocities
          string: /wheel_velocities
        playback/resend:
          name: playback/resend
          bool: false
          type: value
        directory/current:
          type: range
          name: directory/current
          int: 0
          min: 0
          max: 1
          step: 1
        bag/loop:
          bool: false
          name: bag/loop
          type: value
        directory/latch:
          type: value
          name: directory/latch
          bool: false
        directory/play:
          bool: true
          type: value
          name: directory/play
    z: 643
    uuid: csapex::FileImporter_0
    type: csapex::FileImporter
    color:
      - -1
      - -1
      - -1
  - state:
      persistent_params:
        []
      params:
        field:
          type: set
          name: field
          txt: intensity
          string: intensity
        keep organized:
          type: value
          name: keep organized
          bool: false
        interval:
          step: 0.01
          type: interval
          name: interval
          double:
            - 160
            - 1024
          max: 1024
          min: 0
    flipped: false
    z: 486
    minimized: false
    enabled: true
    pos:
      - -49
      - 31
    color:
      - -1
      - -1
      - -1
    type: csapex::PassThrough
    uuid: csapex::PassThrough_0
    label: csapex::PassThrough_0
  - z: 575
    minimized: false
    enabled: true
    type: csapex::PillarLocalization
    uuid: csapex::PillarLocalization_0
    label: csapex::PillarLocalization_0
    pos:
      - 451
      - 362
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        radius:
          type: range
          name: radius
          double: 0.055
          min: 0.01
          max: 1
          step: 0.001
        min pts:
          type: range
          name: min pts
          int: 3
          min: 1
          max: 100
          step: 1
        distance/2:
          name: distance/2
          double: 8.358000000000001
          min: 0.01
          max: 20
          step: 0.001
          type: range
        distance/1:
          max: 20
          step: 0.001
          type: range
          name: distance/1
          double: 9.832000000000001
          min: 0.01
        threshold:
          double: 0.2
          min: 0
          max: 1
          step: 0.001
          type: range
          name: threshold
        cell/resolution:
          name: cell/resolution
          double: 0.05
          min: 0.01
          max: 0.5
          step: 0.01
          type: range
        reset:
          type: trigger
          name: reset
        size:
          int: 28
          min: 1
          step: 1
          max: 50
          type: range
          name: size
        distance/3:
          min: 0.01
          max: 20
          step: 0.001
          type: range
          name: distance/3
          double: 5.572
    flipped: false
  - minimized: false
    enabled: true
    flipped: false
    type: csapex::ClusterPointcloud
    uuid: csapex::ClusterPointcloud_0
    label: csapex::ClusterPointcloud_0
    pos:
      - 50
      - 318
    state:
      params:
        ClusterTolerance:
          type: range
          name: ClusterTolerance
          double: 0.4
          min: 0.001
          max: 2
          step: 0.001
        MaxClusterSize:
          type: range
          name: MaxClusterSize
          int: 10000
          min: 0
          max: 100000
          step: 1000
        MinClusterSize:
          type: range
          name: MinClusterSize
          int: 0
          min: 0
          max: 20000
          step: 200
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    z: 485
  - type: csapex::ParamEstimator
    uuid: csapex::ParamEstimator_0
    label: csapex::ParamEstimator_0
    pos:
      - 449
      - 784
    color:
      - -1
      - -1
      - -1
    enabled: true
    flipped: false
    z: 596
    minimized: false
    state:
      persistent_params:
        []
      params:
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.7889999999999999
          min: 0.5
          max: 1
          step: 0.001
        reset:
          type: trigger
          name: reset
        x_icr:
          type: range
          step: 0.001
          name: x_icr
          interactive: true
          double: -0.02500000000000001
          min: -0.1
          max: 0
        y_icr_l:
          type: range
          step: 0.001
          name: y_icr_l
          interactive: true
          double: 0.33
          min: 0.001
          max: 0.75
        y_icr_r:
          type: range
          name: y_icr_r
          min: -0.75
          interactive: true
          double: -0.5509999999999999
          max: -0.001
          step: 0.001
        alpha_l:
          step: 0.001
          type: range
          name: alpha_l
          interactive: true
          double: 0.515
          min: 0.5
          max: 1
  - pos:
      - 545
      - 1184
    color:
      - -1
      - -1
      - -1
    z: 580
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
    type: csapex::PoseDifference
    uuid: csapex::PoseDifference_0
    label: csapex::PoseDifference_0
  - state:
      persistent_params:
        - alpha_l
        - alpha_r
        - x_icr
        - y_icr_l
        - y_icr_r
      params:
        finish:
          type: trigger
          name: finish
        method:
          type: value
          name: method
          string: default
        y_icr_l:
          type: range
          name: y_icr_l
          interactive: true
          double: 0.33
          min: 0.001
          max: 0.75
          step: 0.001
        server port:
          type: value
          name: server port
          string: 2342
        alpha_r:
          type: range
          name: alpha_r
          interactive: true
          double: 0.7889999999999999
          min: 0.5
          step: 0.001
          max: 1
        alpha_l:
          max: 1
          step: 0.001
          type: range
          name: alpha_l
          interactive: true
          double: 0.515
          min: 0.5
        server name:
          string: localhost
          type: value
          name: server name
        y_icr_r:
          type: range
          name: y_icr_r
          interactive: true
          double: -0.5509999999999999
          min: -0.75
          max: -0.001
          step: 0.001
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
          interactive: true
          double: -0.02500000000000001
          min: -0.1
          max: 0
          step: 0.001
          type: range
          name: x_icr
    label: csapex::EvaOptimizer_0
    pos:
      - -524
      - 294
    color:
      - -1
      - -1
      - -1
    z: 638
    minimized: false
    enabled: true
    flipped: false
    type: csapex::EvaOptimizer
    uuid: csapex::EvaOptimizer_0
  - enabled: true
    flipped: false
    color:
      - -1
      - -1
      - -1
    z: 621
    minimized: false
    type: csapex::Delay
    uuid: csapex::Delay_1
    label: csapex::Delay_1
    pos:
      - -934
      - 786
    state:
      params:
        delay progress:
          type: progress
          name: delay progress
        delay:
          double: 1
          min: 0
          max: 10
          step: 0.01
          type: range
          name: delay
      persistent_params:
        []
  - state:
      persistent_params:
        []
      params:
        ~plot/line/width:
          type: range
          name: ~plot/line/width
          double: 0
          min: 0
          max: 10
          step: 0.01
        ~plot/color/line:
          type: color
          name: ~plot/color/line
          values:
            - 100
            - 100
            - 255
        ~plot/color/background:
          type: color
          name: ~plot/color/background
          values:
            - 255
            - 255
            - 255
        ~plot/width:
          int: 640
          min: 128
          max: 4096
          step: 1
          type: range
          name: ~plot/width
        ~plot/height:
          type: range
          name: ~plot/height
          int: 320
          min: 128
          step: 1
          max: 4096
        reset:
          type: trigger
          name: reset
        ~output/time/relative:
          type: value
          name: ~output/time/relative
          bool: true
        ~output/time/seconds:
          type: value
          name: ~output/time/seconds
          bool: true
        ~plot/color/fill:
          type: color
          name: ~plot/color/fill
          values:
            - 200
            - 200
            - 200
    flipped: false
    color:
      - -1
      - -1
      - -1
    z: 644
    minimized: false
    enabled: true
    type: csapex::TimePlot
    uuid: csapex::TimePlot_1
    label: csapex::TimePlot_1
    pos:
      - 87
      - 1333
forward_out:
  []
adapters:
  []
connections:
  - uuid: csapex::PillarLocalization_0:|:out_1
    targets:
      - csapex::PoseDifference_0:|:in_1
  - uuid: csapex::PoseDifference_0:|:out_1
    targets:
      - csapex::EvaOptimizer_0:|:in_0
  - targets:
      - csapex::TimePlot_1:|:in_0
    uuid: csapex::EvaOptimizer_0:|:out_1
  - uuid: csapex::FileImporter_0:|:trigger_1
    targets:
      - csapex::Delay_1:|:slot_0
  - uuid: csapex::EvaOptimizer_0:|:out_x_icr
    targets:
      - csapex::ParamEstimator_0:|:in_x_icr
  - uuid: csapex::EvaOptimizer_0:|:trigger_4
    targets:
      - csapex::FileImporter_0:|:slot_0
  - uuid: csapex::EvaOptimizer_0:|:out_y_icr_r
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_r
  - uuid: csapex::EvaOptimizer_0:|:out_alpha_l
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_l
  - uuid: csapex::FileImporter_0:|:out_2
    targets:
      - csapex::PassThrough_0:|:in_0
  - uuid: csapex::Delay_1:|:trigger_0
    targets:
      - csapex::EvaOptimizer_0:|:slot_1
  - uuid: csapex::EvaOptimizer_0:|:trigger_0
    targets:
      - csapex::FileImporter_0:|:slot_1
  - targets:
      - csapex::ParamEstimator_0:|:slot_0
    uuid: csapex::PillarLocalization_0:|:trigger_0
  - uuid: csapex::FileImporter_0:|:out_3
    targets:
      - csapex::ParamEstimator_0:|:in_0
  - uuid: csapex::EvaOptimizer_0:|:out_y_icr_l
    targets:
      - csapex::ParamEstimator_0:|:in_y_icr_l
  - uuid: csapex::EvaOptimizer_0:|:out_alpha_r
    targets:
      - csapex::ParamEstimator_0:|:in_alpha_r
  - targets:
      - csapex::PoseDifference_0:|:in_0
    uuid: csapex::ParamEstimator_0:|:out_0
  - uuid: csapex::FileImporter_0:|:out_0
    targets:
      - csapex::ParamEstimator_0:|:in_1
  - uuid: csapex::ClusterPointcloud_0:|:out_0
    targets:
      - csapex::PillarLocalization_0:|:in_1
  - uuid: csapex::PassThrough_0:|:out_0
    targets:
      - csapex::PillarLocalization_0:|:in_0
      - csapex::ClusterPointcloud_0:|:in_0
forward_in:
  []