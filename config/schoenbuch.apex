#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  assignments:
    - id: 1
      uuid: ~
    - id: 1
      uuid: csapex::IMUIntegration_0
    - uuid: csapex::ImportRos_2
      id: 1
    - id: 1
      uuid: csapex::TransformPublisher_0
    - id: 1
      uuid: csapex::TransformInverter_1
    - uuid: csapex::DynamicTransform_0
      id: 1
    - id: 1
      uuid: csapex::ImportRos_3
    - id: 0
      uuid: csapex::VelodyneUndistortion_0
    - id: 1
      uuid: csapex::ExportRos_0
    - id: 1
      uuid: csapex::ImportRos_4
    - id: 0
      uuid: csapex::VelodyneSegmentation_0
    - id: 1
      uuid: csapex::ExportRos_5
    - uuid: csapex::ExportRos_6
      id: 1
    - id: 0
      uuid: csapex::ClusterPointcloud_1
    - id: 1
      uuid: csapex::LabelClusteredPointCloud_0
    - id: 1
      uuid: csapex::ColorPointCloud_0
    - uuid: csapex::ExportRos_7
      id: 1
    - uuid: csapex::ClustersToMarkerArray_0
      id: 1
    - id: 0
      uuid: schoenbuch::LocalHazardField_0
    - uuid: csapex::DynamicTransform_1
      id: 1
    - uuid: csapex::ExportRos_8
      id: 1
    - uuid: csapex::ExportRos_9
      id: 1
  groups:
    - id: 2
      name: Imu Tf
nodes:
  - type: csapex::IMUIntegration
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        drift/yaw:
          name: drift/yaw
          min: -1.1
          step: 0.0001
          double: 0.000299999999999967
          max: 1.1
          type: range
        offset/roll:
          name: offset/roll
          min: -3.141592653589793
          max: 3.141592653589793
          type: angle
          value: -0.06108652381980173
        offset/pitch:
          max: 3.141592653589793
          name: offset/pitch
          min: -3.141592653589793
          type: angle
          value: 0.09336
        reset:
          name: reset
          type: trigger
    color:
      - -1
      - -1
      - -1
    label: csapex::IMUIntegration_0
    pos:
      - -439
      - -247
    minimized: false
    z: 740
    uuid: csapex::IMUIntegration_0
  - enabled: true
    minimized: false
    flipped: false
    pos:
      - -902
      - -170
    z: 737
    type: csapex::ImportRos
    uuid: csapex::ImportRos_2
    state:
      params:
        topic:
          type: set
          name: topic
          txt: /odom
          string: /odom
        latch:
          type: value
          name: latch
          bool: false
        buffer/max_wait:
          min: 0
          name: buffer/max_wait
          double: 1
          step: 0.1
          type: range
          max: 10
        refresh:
          type: trigger
          name: refresh
        buffer/length:
          type: range
          name: buffer/length
          double: 1
          step: 0.1
          max: 10
          min: 0
        queue:
          max: 30
          type: range
          name: queue
          step: 1
          int: 0
          min: 0
        rate:
          double: 100
          type: range
          min: 0.1
          step: 0.1
          max: 100
          name: rate
      persistent_params:
        []
    label: csapex::ImportRos_2
    color:
      - -1
      - -1
      - -1
  - uuid: csapex::TransformPublisher_0
    minimized: false
    label: csapex::TransformPublisher_0
    color:
      - -1
      - -1
      - -1
    type: csapex::TransformPublisher
    pos:
      - 158
      - -337
    enabled: true
    z: 672
    flipped: false
    state:
      params:
        from:
          string: ""
          name: from
          type: value
        to:
          type: value
          string: ""
          name: to
      persistent_params:
        []
  - uuid: csapex::TransformInverter_1
    flipped: false
    label: csapex::TransformInverter_1
    enabled: true
    pos:
      - -191
      - -426
    z: 56
    state:
      persistent_params:
        []
      params:
        {}
    minimized: false
    color:
      - -1
      - -1
      - -1
    type: csapex::TransformInverter
  - minimized: false
    enabled: true
    flipped: false
    color:
      - -1
      - -1
      - -1
    uuid: csapex::DynamicTransform_0
    z: 736
    pos:
      - -870
      - 160
    type: csapex::DynamicTransform
    label: csapex::DynamicTransform_0
    state:
      persistent_params:
        []
      params:
        reset tf:
          name: reset tf
          type: trigger
        refresh:
          name: refresh
          type: trigger
        target:
          type: set
          txt: /imu_link
          string: /imu_link
          name: target
        source:
          type: set
          txt: /base_link
          string: /base_link
          name: source
  - uuid: csapex::ImportRos_3
    type: csapex::ImportRos
    color:
      - -1
      - -1
      - -1
    flipped: false
    state:
      params:
        refresh:
          type: trigger
          name: refresh
        buffer/max_wait:
          max: 10
          type: range
          double: 1
          name: buffer/max_wait
          step: 0.1
          min: 0
        topic:
          txt: /velodyne_points
          string: /velodyne_points
          type: set
          name: topic
        latch:
          bool: false
          name: latch
          type: value
        buffer/length:
          step: 0.1
          double: 1
          name: buffer/length
          min: 0
          max: 10
          type: range
        queue:
          max: 30
          min: 0
          step: 1
          name: queue
          int: 0
          type: range
        rate:
          type: range
          name: rate
          min: 0.1
          double: 100
          max: 100
          step: 0.1
      persistent_params:
        []
    minimized: false
    enabled: true
    label: csapex::ImportRos_3
    pos:
      - -723
      - 922
    z: 473
  - pos:
      - -335
      - 921
    state:
      params:
        scan/offset:
          name: scan/offset
          type: value
          double: 0.06
        scan/duration:
          type: value
          name: scan/duration
          double: 0.095
      persistent_params:
        []
    enabled: true
    type: csapex::VelodyneUndistortion
    uuid: csapex::VelodyneUndistortion_0
    label: csapex::VelodyneUndistortion_0
    color:
      - -1
      - -1
      - -1
    minimized: false
    flipped: false
    z: 738
  - flipped: false
    uuid: csapex::ExportRos_0
    enabled: true
    color:
      - -1
      - -1
      - -1
    state:
      params:
        topic:
          name: topic
          string: /velodyne_points/undistorted
          type: value
        queue:
          int: 1
          type: range
          step: 1
          max: 32
          name: queue
          min: 1
      persistent_params:
        []
    z: 694
    label: csapex::ExportRos_0
    pos:
      - -345
      - 1176
    type: csapex::ExportRos
    minimized: false
  - state:
      persistent_params:
        []
      params:
        buffer/length:
          step: 0.1
          max: 10
          name: buffer/length
          type: range
          double: 1
          min: 0
        refresh:
          type: trigger
          name: refresh
        buffer/max_wait:
          name: buffer/max_wait
          step: 0.1
          type: range
          max: 10
          min: 0
          double: 1
        topic:
          name: topic
          type: set
          txt: /imu
          string: /imu
        rate:
          type: range
          max: 100
          step: 0.1
          name: rate
          double: 100
          min: 0.1
        queue:
          max: 30
          min: 0
          type: range
          name: queue
          int: 0
          step: 1
        latch:
          type: value
          name: latch
          bool: false
    z: 213
    color:
      - -1
      - -1
      - -1
    pos:
      - -950
      - -486
    uuid: csapex::ImportRos_4
    label: csapex::ImportRos_4
    flipped: false
    enabled: true
    type: csapex::ImportRos
    minimized: false
  - z: 741
    minimized: false
    color:
      - -1
      - -1
      - -1
    flipped: false
    enabled: true
    pos:
      - 79
      - 740
    type: csapex::VelodyneSegmentation
    uuid: csapex::VelodyneSegmentation_0
    label: csapex::VelodyneSegmentation_0
    state:
      params:
        max floor angle:
          type: angle
          name: max floor angle
          value: 0.3054326190990078
          min: -3.141592653589793
          max: 3.141592653589793
        max floor height:
          step: 0.01
          double: -0.25
          max: 5
          min: -5
          type: range
          name: max floor height
        min obstacle angle:
          min: -3.141592653589793
          max: 3.141592653589793
          name: min obstacle angle
          value: 0.9032078879070662
          type: angle
      persistent_params:
        []
  - type: csapex::ExportRos
    z: 676
    enabled: true
    flipped: false
    pos:
      - 537
      - 926
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        topic:
          type: value
          name: topic
          string: /visualization_marker_array
        queue:
          max: 32
          step: 1
          min: 1
          type: range
          name: queue
          int: 1
    minimized: false
    uuid: csapex::ExportRos_5
    label: csapex::ExportRos_5
  - type: csapex::ExportRos
    color:
      - -1
      - -1
      - -1
    label: csapex::ExportRos_6
    pos:
      - 779
      - 173
    minimized: false
    z: 428
    uuid: csapex::ExportRos_6
    state:
      params:
        topic:
          type: value
          name: topic
          string: objects
        queue:
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
      persistent_params:
        []
    enabled: true
    flipped: false
  - type: csapex::ClusterPointcloud
    z: 684
    enabled: true
    minimized: false
    pos:
      - 466
      - 514
    color:
      - -1
      - -1
      - -1
    flipped: false
    state:
      params:
        radius:
          type: range
          name: radius
          double: 100
          min: 0
          max: 1000
          step: 0.1
        MinClusterSize:
          type: range
          max: 20000
          step: 1
          name: MinClusterSize
          int: 20
          min: 0
        ClusterTolerance:
          type: range
          name: ClusterTolerance
          double: 0.175
          max: 2
          min: 0.001
          step: 0.001
        opening_angle:
          value: 0.1701696020694472
          min: -3.141592653589793
          max: 3.141592653589793
          type: angle
          name: opening_angle
        MaxClusterSize:
          min: 0
          max: 100000
          type: range
          name: MaxClusterSize
          int: 10000
          step: 1
        method:
          txt: POLAR
          int: 1
          type: set
          name: method
      persistent_params:
        []
    uuid: csapex::ClusterPointcloud_1
    label: csapex::ClusterPointcloud_1
  - z: 430
    uuid: csapex::LabelClusteredPointCloud_0
    label: csapex::LabelClusteredPointCloud_0
    pos:
      - 359
      - 354
    state:
      persistent_params:
        []
      params:
        {}
    color:
      - -1
      - -1
      - -1
    flipped: false
    type: csapex::LabelClusteredPointCloud
    minimized: false
    enabled: true
  - type: csapex::ColorPointCloud
    z: 431
    minimized: false
    uuid: csapex::ColorPointCloud_0
    label: csapex::ColorPointCloud_0
    pos:
      - 395
      - 242
    state:
      params:
        {}
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    enabled: true
    flipped: false
  - color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        topic:
          type: value
          name: topic
          string: /visualization_marker_array
        queue:
          name: queue
          int: 1
          min: 1
          max: 32
          step: 1
          type: range
    uuid: csapex::ExportRos_7
    label: csapex::ExportRos_7
    pos:
      - 1130
      - 331
    enabled: true
    flipped: false
    type: csapex::ExportRos
    z: 551
    minimized: false
  - uuid: csapex::ClustersToMarkerArray_0
    label: csapex::ClustersToMarkerArray_0
    pos:
      - 923
      - 501
    flipped: false
    state:
      params:
        PCA:
          bool: true
          type: value
          name: PCA
      persistent_params:
        []
    type: csapex::ClustersToMarkerArray
    minimized: false
    enabled: true
    color:
      - -1
      - -1
      - -1
    z: 686
  - state:
      persistent_params:
        []
      params:
        cell_resolution:
          step: 0.1
          type: range
          name: cell_resolution
          double: 1
          max: 10
          min: 0.1
        probability/free:
          type: range
          name: probability/free
          double: 0.01
          min: 0.01
          max: 0.99
          step: 0.01
        probability/measurement:
          min: 0.01
          type: range
          name: probability/measurement
          double: 0.68
          max: 0.99
          step: 0.01
        resolution:
          min: 0.01
          max: 1
          step: 0.01
          type: range
          name: resolution
          double: 0.09999999999999999
        probability/decay_rate:
          step: 1
          type: range
          name: probability/decay_rate
          int: 2
          min: 0
          max: 100
        range:
          type: range
          name: range
          double: 10
          min: 0.2
          max: 10
          step: 0.1
        probability/decay_min_odds:
          double: 0.26
          min: 0.01
          step: 0.01
          max: 0.99
          type: range
          name: probability/decay_min_odds
        size:
          type: range
          name: size
          double: 20
          max: 20
          min: 0.01
          step: 0.01
        probability/decay:
          max: 0.99
          step: 0.01
          type: range
          name: probability/decay
          double: 0.24
          min: 0.01
        probability/threshold:
          step: 0.01
          type: range
          name: probability/threshold
          double: 0.92
          min: 0
          max: 1
    uuid: schoenbuch::LocalHazardField_0
    label: schoenbuch::LocalHazardField_0
    pos:
      - 1249
      - 691
    color:
      - -1
      - -1
      - -1
    z: 683
    minimized: false
    enabled: true
    flipped: false
    type: schoenbuch::LocalHazardField
  - type: csapex::DynamicTransform
    z: 607
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        source:
          type: set
          name: source
          txt: /base_link
          string: /base_link
        target:
          type: set
          name: target
          txt: /odom
          string: /odom
        refresh:
          type: trigger
          name: refresh
        reset tf:
          type: trigger
          name: reset tf
      persistent_params:
        []
    uuid: csapex::DynamicTransform_1
    label: csapex::DynamicTransform_1
    pos:
      - 863
      - 1106
    color:
      - -1
      - -1
      - -1
  - enabled: true
    flipped: false
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
    type: csapex::ExportRos
    uuid: csapex::ExportRos_8
    label: csapex::ExportRos_8
    pos:
      - 1686
      - 1015
    color:
      - -1
      - -1
      - -1
    z: 680
    minimized: false
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_9
    label: csapex::ExportRos_9
    pos:
      - 1668
      - 1174
    color:
      - -1
      - -1
      - -1
    z: 599
    minimized: false
    enabled: true
    flipped: false
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
adapters:
  []
connections:
  - uuid: event_ros_time_reset
    targets:
      - csapex::IMUIntegration_0:|:slot_0
    types:
      - default
  - uuid: schoenbuch::LocalHazardField_0:|:out_1
    targets:
      - csapex::ExportRos_9:|:in_0
    types:
      - default
  - uuid: csapex::VelodyneSegmentation_0:|:out_1
    targets:
      - csapex::ExportRos_5:|:in_0
    types:
      - default
  - uuid: csapex::IMUIntegration_0:|:out_0
    targets:
      - csapex::TransformInverter_1:|:in_0
    types:
      - default
  - uuid: csapex::LabelClusteredPointCloud_0:|:out_0
    targets:
      - csapex::ColorPointCloud_0:|:in_0
    types:
      - default
  - uuid: csapex::ColorPointCloud_0:|:out_0
    targets:
      - csapex::ExportRos_6:|:in_0
    types:
      - default
  - uuid: csapex::VelodyneUndistortion_0:|:out_0
    targets:
      - csapex::ExportRos_0:|:in_0
      - csapex::VelodyneSegmentation_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::ImportRos_3:|:out_0
    targets:
      - csapex::VelodyneUndistortion_0:|:in_0
    types:
      - default
  - uuid: csapex::ImportRos_4:|:out_0
    targets:
      - csapex::IMUIntegration_0:|:in_0
    types:
      - default
  - uuid: csapex::ClusterPointcloud_1:|:out_0
    targets:
      - csapex::LabelClusteredPointCloud_0:|:in_1
      - csapex::ClustersToMarkerArray_0:|:in_1
    types:
      - default
      - default
  - uuid: csapex::VelodyneSegmentation_0:|:out_0
    targets:
      - csapex::ClusterPointcloud_1:|:in_0
      - csapex::LabelClusteredPointCloud_0:|:in_0
      - csapex::ClustersToMarkerArray_0:|:in_0
      - schoenbuch::LocalHazardField_0:|:in_0
    types:
      - default
      - default
      - default
      - default
  - uuid: schoenbuch::LocalHazardField_0:|:out_0
    targets:
      - csapex::ExportRos_8:|:in_0
    types:
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
  - uuid: csapex::DynamicTransform_0:|:out_0
    targets:
      - csapex::IMUIntegration_0:|:in_2
    types:
      - default
  - uuid: csapex::ClustersToMarkerArray_0:|:out_0
    targets:
      - csapex::ExportRos_7:|:in_0
    types:
      - default
  - uuid: csapex::DynamicTransform_1:|:out_0
    targets:
      - schoenbuch::LocalHazardField_0:|:in_1
    types:
      - default
uuid_map:
  csapex::ExtractTimeStamp: 2
  csapex::ColorPointCloud: 1
  csapex::ImportRos: 5
  csapex::CloudRenderer: 3
  csapex::ClusterPointcloud: 2
  csapex::VelodyneUndistortion: 1
  csapex::LabelClusteredPointCloud: 1
  csapex::Note: 1
  csapex::ObjectDetector: 1
  csapex::TransformPublisher: 3
  csapex::VelodyneSegmentation: 1
  csapex::ClustersToMarkerArray: 1
  csapex::EvaOptimizer: 1
  csapex::Clock: 1
  csapex::TextDisplay: 16
  csapex::IMUIntegration: 1
  csapex::TransformInverter: 3
  csapex::DynamicTransform: 2
  csapex::TransformCloud: 1
  schoenbuch::LocalHazardField: 1
  csapex::LoamSegmentation: 1
  csapex::StaticTransform: 2
  csapex::ExportRos: 10
  schoenbuch::PersonDetector: 1