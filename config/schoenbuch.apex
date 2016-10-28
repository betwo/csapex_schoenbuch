#!/localhome/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups:
    - name: Imu Tf
      id: 2
  assignments:
    - id: 1
      uuid: ~
    - uuid: csapex::IMUIntegration_0
      id: 1
    - id: 1
      uuid: csapex::ImportRos_2
    - uuid: csapex::TransformInverter_1
      id: 1
    - id: 1
      uuid: csapex::TransformPublisher_0
    - id: 1
      uuid: csapex::ImportRos_3
    - uuid: csapex::ExportRos_0
      id: 1
    - uuid: csapex::VelodyneUndistortion_0
      id: 1
    - uuid: csapex::ImportRos_4
      id: 1
    - id: 1
      uuid: csapex::ExportRos_9
    - uuid: csapex::ExportRos_8
      id: 1
    - id: 1
      uuid: csapex::StaticTransform_2
    - uuid: csapex::TransformPublisher_3
      id: 1
    - uuid: csapex::CropBox_0
      id: 1
    - uuid: csapex::ExportRos_10
      id: 1
    - uuid: csapex::ExtractMeasures_0
      id: 1
    - uuid: csapex::VoxelGrid_0
      id: 1
    - id: 1
      uuid: csapex::Note_1
nodes:
  - type: csapex::IMUIntegration
    uuid: csapex::IMUIntegration_0
    label: csapex::IMUIntegration_0
    pos:
      - -439
      - -247
    color:
      - -1
      - -1
      - -1
    z: 1038
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        acc/decay:
          type: range
          name: acc/decay
          double: 0.925
          min: 0
          max: 1
          step: 1e-05
        acc/gravity:
          type: range
          name: acc/gravity
          double: 9.810090000000001
          min: 9.5
          max: 10
          step: 1e-05
        acc/queue:
          type: range
          name: acc/queue
          int: 3
          min: 1
          max: 32
          step: 1
        drift/yaw:
          type: range
          name: drift/yaw
          double: -0.00043
          min: -0.001
          max: 0
          step: 1e-08
        offset/pitch:
          type: angle
          name: offset/pitch
          value: 0.01
          min: -3.141592653589793
          max: 3.141592653589793
        offset/roll:
          type: angle
          name: offset/roll
          value: 0
          min: -3.141592653589793
          max: 3.141592653589793
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
  - pos:
      - -902
      - -170
    label: csapex::ImportRos_2
    uuid: csapex::ImportRos_2
    color:
      - -1
      - -1
      - -1
    z: 942
    minimized: false
    type: csapex::ImportRos
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        buffer/length:
          type: range
          name: buffer/length
          double: 1
          min: 0
          max: 10
          step: 0.1
        buffer/max_wait:
          type: range
          name: buffer/max_wait
          double: 1
          min: 0
          max: 10
          step: 0.1
        latch:
          type: value
          name: latch
          bool: false
        queue:
          type: range
          name: queue
          int: 10
          min: 0
          max: 30
          step: 1
        rate:
          type: range
          name: rate
          double: 100
          min: 0.1
          max: 100
          step: 0.1
        refresh:
          type: trigger
          name: refresh
        topic:
          type: set
          name: topic
          txt: /odom
          string: /odom
      persistent_params:
        []
  - type: csapex::TransformPublisher
    uuid: csapex::TransformPublisher_0
    label: csapex::TransformPublisher_0
    pos:
      - 158
      - -337
    color:
      - -1
      - -1
      - -1
    z: 672
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        from:
          type: value
          name: from
          string: ""
        to:
          type: value
          name: to
          string: ""
      persistent_params:
        []
  - uuid: csapex::TransformInverter_1
    label: csapex::TransformInverter_1
    pos:
      - -191
      - -426
    type: csapex::TransformInverter
    color:
      - -1
      - -1
      - -1
    z: 866
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::ImportRos_3
    label: csapex::ImportRos_3
    type: csapex::ImportRos
    pos:
      - -723
      - 922
    color:
      - -1
      - -1
      - -1
    z: 473
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        buffer/length:
          type: range
          name: buffer/length
          double: 1
          min: 0
          max: 10
          step: 0.1
        buffer/max_wait:
          type: range
          name: buffer/max_wait
          double: 1
          min: 0
          max: 10
          step: 0.1
        latch:
          type: value
          name: latch
          bool: false
        queue:
          type: range
          name: queue
          int: 0
          min: 0
          max: 30
          step: 1
        rate:
          type: range
          name: rate
          double: 100
          min: 0.1
          max: 100
          step: 0.1
        refresh:
          type: trigger
          name: refresh
        topic:
          type: set
          name: topic
          txt: /velodyne_points
          string: /velodyne_points
      persistent_params:
        []
  - uuid: csapex::VelodyneUndistortion_0
    pos:
      - -337
      - 903
    label: csapex::VelodyneUndistortion_0
    color:
      - -1
      - -1
      - -1
    z: 1030
    type: csapex::VelodyneUndistortion
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        scan/duration:
          type: value
          name: scan/duration
          double: 0.095
        scan/offset:
          type: value
          name: scan/offset
          double: 0.06
      persistent_params:
        []
  - uuid: csapex::ExportRos_0
    label: csapex::ExportRos_0
    pos:
      - -345
      - 1176
    type: csapex::ExportRos
    color:
      - -1
      - -1
      - -1
    z: 694
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
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
          string: /velodyne_points/undistorted
      persistent_params:
        []
  - uuid: csapex::ImportRos_4
    label: csapex::ImportRos_4
    pos:
      - -950
      - -486
    type: csapex::ImportRos
    color:
      - -1
      - -1
      - -1
    z: 940
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        buffer/length:
          type: range
          name: buffer/length
          double: 1
          min: 0
          max: 10
          step: 0.1
        buffer/max_wait:
          type: range
          name: buffer/max_wait
          double: 1
          min: 0
          max: 10
          step: 0.1
        latch:
          type: value
          name: latch
          bool: false
        queue:
          type: range
          name: queue
          int: 10
          min: 0
          max: 30
          step: 1
        rate:
          type: range
          name: rate
          double: 100
          min: 0.1
          max: 100
          step: 0.1
        refresh:
          type: trigger
          name: refresh
        topic:
          type: set
          name: topic
          txt: /imu
          string: /imu
      persistent_params:
        []
  - uuid: csapex::ExportRos_8
    label: csapex::ExportRos_8
    pos:
      - 1686
      - 1015
    color:
      - -1
      - -1
      - -1
    z: 680
    type: csapex::ExportRos
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
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
          string: /obstacle_cloud
      persistent_params:
        []
  - uuid: csapex::ExportRos_9
    label: csapex::ExportRos_9
    pos:
      - 1668
      - 1174
    type: csapex::ExportRos
    color:
      - -1
      - -1
      - -1
    z: 599
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
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
          string: /visualization_marker_array
      persistent_params:
        []
  - uuid: csapex::StaticTransform_2
    label: csapex::StaticTransform_2
    pos:
      - -180
      - 72
    type: csapex::StaticTransform
    color:
      - -1
      - -1
      - -1
    z: 871
    minimized: false
    enabled: true
    flipped: true
    exec_mode: 1
    state:
      params:
        child_frame:
          type: value
          name: child_frame
          string: /imu_link
        dx:
          type: value
          name: dx
          double: 0
        dy:
          type: value
          name: dy
          double: 0
        dz:
          type: value
          name: dz
          double: 0
        frame:
          type: value
          name: frame
          string: /base_link
        pitch:
          type: angle
          name: pitch
          value: 0
          min: -3.141592653589793
          max: 3.141592653589793
        roll:
          type: angle
          name: roll
          value: 0
          min: -3.141592653589793
          max: 3.141592653589793
        yaw:
          type: angle
          name: yaw
          value: 1.570796326794897
          min: -3.141592653589793
          max: 3.141592653589793
      persistent_params:
        []
  - type: csapex::TransformPublisher
    uuid: csapex::TransformPublisher_3
    label: csapex::TransformPublisher_3
    pos:
      - -648
      - 491
    color:
      - -1
      - -1
      - -1
    z: 758
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
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
  - type: csapex::CropBox
    uuid: csapex::CropBox_0
    label: csapex::CropBox_0
    pos:
      - 23
      - 1046
    color:
      - -1
      - -1
      - -1
    z: 1045
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        dx:
          type: interval
          name: dx
          double:
            - -10
            - 10
          min: -100
          max: 100
          step: 0.01
        dy:
          type: interval
          name: dy
          double:
            - -10
            - 10
          min: -100
          max: 100
          step: 0.01
        dz:
          type: interval
          name: dz
          double:
            - -100
            - 100
          min: -100
          max: 100
          step: 0.01
        keep organized:
          type: value
          name: keep organized
          bool: true
      persistent_params:
        []
  - uuid: csapex::ExportRos_10
    label: csapex::ExportRos_5
    pos:
      - 702
      - 1169
    color:
      - -1
      - -1
      - -1
    type: csapex::ExportRos
    z: 849
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
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
          string: /near
      persistent_params:
        []
  - label: csapex::ExtractMeasures_0
    uuid: csapex::ExtractMeasures_0
    pos:
      - 344
      - -101
    type: csapex::ExtractMeasures
    color:
      - -1
      - -1
      - -1
    z: 826
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        dx:
          type: value
          name: dx
          double: -0.7373254600926836
        dy:
          type: value
          name: dy
          double: -3.37661252207456
        dz:
          type: value
          name: dz
          double: -0.3898453499967215
        norm:
          type: value
          name: norm
          double: 3.478094356863493
        pitch:
          type: angle
          name: pitch
          interactive: true
          value: 0.01267995104499566
          min: -3.141592653589793
          max: 3.141592653589793
        roll:
          type: angle
          name: roll
          interactive: true
          value: 0.02415906716315791
          min: -3.141592653589793
          max: 3.141592653589793
        yaw:
          type: angle
          name: yaw
          value: -1.476547869159638
          min: -3.141592653589793
          max: 3.141592653589793
        yaw/cap:
          type: value
          name: yaw/cap
          bool: true
      persistent_params:
        []
  - type: csapex::VoxelGrid
    uuid: csapex::VoxelGrid_0
    label: csapex::VoxelGrid_0
    pos:
      - 585
      - 970
    color:
      - -1
      - -1
      - -1
    z: 1044
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        remove NAN:
          type: value
          name: remove NAN
          bool: false
        resolution:
          type: range
          name: resolution
          double: 0.5
          min: 0.01
          max: 1
          step: 0.01
      persistent_params:
        []
  - uuid: csapex::Note_1
    label: prototype
    pos:
      - -719
      - 154
    type: csapex::Note
    color:
      - -1
      - -1
      - -1
    z: -2147483645
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        h:
          type: value
          name: h
          int: 90
        text:
          type: value
          name: text
          string: "Settings for 25-11:\n0,00000\n0,01000\n-0,00043"
        w:
          type: value
          name: w
          int: 180
      persistent_params:
        []
uuid_map:
  csapex::Clock: 1
  csapex::ClusterPointcloud: 2
  csapex::CloudRenderer: 3
  csapex::ClustersToMarkerArray: 1
  csapex::StaticTransform: 3
  csapex::TextDisplay: 19
  csapex::TimePlot: 2
  csapex::TransformCloud: 1
  csapex::TransformInverter: 3
  csapex::TransformPublisher: 4
  csapex::VelodyneSegmentation: 1
  csapex::VelodyneUndistortion: 1
  schoenbuch::LocalHazardField: 1
  schoenbuch::PersonDetector: 1
  csapex::VoxelGrid: 1
  csapex::CropBox: 1
  csapex::DynamicTransform: 2
  csapex::EvaOptimizer: 1
  csapex::ExportRos: 11
  csapex::ExtractMeasures: 1
  csapex::ExtractTimeStamp: 2
  csapex::IMUIntegration: 1
  csapex::ImportRos: 5
  csapex::LabelClusteredPointCloud: 1
  csapex::LoamSegmentation: 1
  csapex::Note: 2
  csapex::ObjectDetector: 1
  csapex::ColorPointCloud: 1
connections:
  - targets:
      - csapex::ExportRos_0:|:in_0
      - csapex::CropBox_0:|:in_0
    types:
      - default
      - default
    uuid: csapex::VelodyneUndistortion_0:|:out_0
  - uuid: csapex::IMUIntegration_0:|:out_0
    targets:
      - csapex::TransformInverter_1:|:in_0
      - csapex::ExtractMeasures_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::TransformInverter_1:|:out_0
    targets:
      - csapex::TransformPublisher_0:|:in_0
    types:
      - default
  - uuid: csapex::ImportRos_2:|:out_0
    targets:
      - csapex::IMUIntegration_0:|:in_1
    types:
      - default
  - uuid: csapex::StaticTransform_2:|:out_0
    targets:
      - csapex::TransformPublisher_3:|:in_0
      - csapex::IMUIntegration_0:|:in_2
    types:
      - default
      - default
  - uuid: csapex::ImportRos_4:|:out_0
    targets:
      - csapex::IMUIntegration_0:|:in_0
    types:
      - default
  - uuid: csapex::ImportRos_3:|:out_0
    targets:
      - csapex::VelodyneUndistortion_0:|:in_0
    types:
      - default
  - uuid: csapex::CropBox_0:|:out_0
    targets:
      - csapex::VoxelGrid_0:|:in_0
    types:
      - default
  - uuid: csapex::VoxelGrid_0:|:out_0
    targets:
      - csapex::ExportRos_10:|:in_0
    types:
      - default
  - uuid: event_ros_time_reset
    targets:
      - csapex::IMUIntegration_0:|:slot_0
    types:
      - default
adapters:
  []