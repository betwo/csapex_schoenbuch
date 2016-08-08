#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups:
    - id: 2
      name: Imu Tf
  assignments:
    - uuid: ~
      id: 1
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
    - uuid: csapex::ImportRos_3
      id: 1
    - uuid: csapex::VelodyneUndistortion_0
      id: 0
    - id: 1
      uuid: csapex::ExportRos_0
    - uuid: csapex::ImportRos_4
      id: 1
    - uuid: csapex::VelodyneSegmentation_0
      id: 0
    - uuid: csapex::ExportRos_5
      id: 1
    - uuid: csapex::ExportRos_6
      id: 1
    - uuid: csapex::ClusterPointcloud_1
      id: 0
    - id: 1
      uuid: csapex::LabelClusteredPointCloud_0
    - uuid: csapex::ColorPointCloud_0
      id: 1
    - uuid: csapex::ExportRos_7
      id: 1
    - uuid: csapex::ClustersToMarkerArray_0
      id: 1
    - uuid: schoenbuch::LocalHazardField_0
      id: 0
    - uuid: csapex::DynamicTransform_1
      id: 1
    - uuid: csapex::ExportRos_8
      id: 1
    - uuid: csapex::ExportRos_9
      id: 1
adapters:
  []
connections:
  - targets:
      - csapex::IMUIntegration_0:|:slot_0
    uuid: event_ros_time_reset
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
  - uuid: csapex::ClusterPointcloud_1:|:out_0
    targets:
      - csapex::LabelClusteredPointCloud_0:|:in_1
      - csapex::ClustersToMarkerArray_0:|:in_1
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
  - uuid: csapex::VelodyneUndistortion_0:|:out_0
    targets:
      - csapex::ExportRos_0:|:in_0
      - csapex::VelodyneSegmentation_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::ColorPointCloud_0:|:out_0
    targets:
      - csapex::ExportRos_6:|:in_0
    types:
      - default
  - uuid: csapex::LabelClusteredPointCloud_0:|:out_0
    targets:
      - csapex::ColorPointCloud_0:|:in_0
    types:
      - default
  - uuid: csapex::IMUIntegration_0:|:out_0
    targets:
      - csapex::TransformInverter_1:|:in_0
    types:
      - default
  - uuid: csapex::DynamicTransform_1:|:out_0
    targets:
      - schoenbuch::LocalHazardField_0:|:in_1
    types:
      - default
  - uuid: csapex::ClustersToMarkerArray_0:|:out_0
    targets:
      - csapex::ExportRos_7:|:in_0
    types:
      - default
  - uuid: csapex::DynamicTransform_0:|:out_0
    targets:
      - csapex::IMUIntegration_0:|:in_2
    types:
      - default
  - uuid: csapex::ImportRos_2:|:out_0
    targets:
      - csapex::IMUIntegration_0:|:in_1
    types:
      - default
  - uuid: csapex::TransformInverter_1:|:out_0
    targets:
      - csapex::TransformPublisher_0:|:in_0
    types:
      - default
  - uuid: schoenbuch::LocalHazardField_0:|:out_0
    targets:
      - csapex::ExportRos_8:|:in_0
    types:
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
nodes:
  - enabled: true
    flipped: false
    type: csapex::IMUIntegration
    uuid: csapex::IMUIntegration_0
    state:
      params:
        drift/yaw:
          type: range
          name: drift/yaw
          double: 0.000299999999999967
          min: -1.1
          max: 1.1
          step: 0.0001
        offset/pitch:
          type: angle
          name: offset/pitch
          value: 0.09336
          min: -3.141592653589793
          max: 3.141592653589793
        offset/roll:
          type: angle
          name: offset/roll
          value: -0.06108652381980173
          min: -3.141592653589793
          max: 3.141592653589793
        reset:
          type: trigger
          name: reset
      persistent_params:
        []
    label: csapex::IMUIntegration_0
    pos:
      - -439
      - -247
    color:
      - -1
      - -1
      - -1
    z: 740
    minimized: false
  - color:
      - -1
      - -1
      - -1
    z: 737
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
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
        buffer/length:
          type: range
          name: buffer/length
          double: 1
          min: 0
          max: 10
          step: 0.1
        refresh:
          type: trigger
          name: refresh
        topic:
          type: set
          name: topic
          txt: /odom
          string: /odom
        buffer/max_wait:
          type: range
          name: buffer/max_wait
          double: 1
          min: 0
          max: 10
          step: 0.1
    type: csapex::ImportRos
    uuid: csapex::ImportRos_2
    label: csapex::ImportRos_2
    pos:
      - -902
      - -170
  - z: 672
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        from:
          type: value
          name: from
          string: ""
        to:
          type: value
          name: to
          string: ""
    type: csapex::TransformPublisher
    uuid: csapex::TransformPublisher_0
    label: csapex::TransformPublisher_0
    pos:
      - 158
      - -337
    color:
      - -1
      - -1
      - -1
  - type: csapex::TransformInverter
    uuid: csapex::TransformInverter_1
    label: csapex::TransformInverter_1
    pos:
      - -191
      - -426
    color:
      - -1
      - -1
      - -1
    z: 56
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        {}
  - label: csapex::DynamicTransform_0
    pos:
      - -870
      - 160
    color:
      - -1
      - -1
      - -1
    z: 736
    minimized: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        refresh:
          type: trigger
          name: refresh
        reset tf:
          type: trigger
          name: reset tf
        source:
          type: set
          name: source
          txt: /base_link
          string: /base_link
        target:
          type: set
          name: target
          txt: /imu_link
          string: /imu_link
    type: csapex::DynamicTransform
    uuid: csapex::DynamicTransform_0
  - type: csapex::ImportRos
    uuid: csapex::ImportRos_3
    label: csapex::ImportRos_3
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
  - type: csapex::VelodyneUndistortion
    uuid: csapex::VelodyneUndistortion_0
    label: csapex::VelodyneUndistortion_0
    pos:
      - -335
      - 921
    color:
      - -1
      - -1
      - -1
    z: 738
    minimized: false
    enabled: true
    flipped: false
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
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_0
    label: csapex::ExportRos_0
    pos:
      - -345
      - 1176
    color:
      - -1
      - -1
      - -1
    z: 694
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
          string: /velodyne_points/undistorted
      persistent_params:
        []
  - state:
      params:
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
          step: 1
          type: range
          name: queue
          int: 0
          min: 0
          max: 30
      persistent_params:
        []
    type: csapex::ImportRos
    uuid: csapex::ImportRos_4
    label: csapex::ImportRos_4
    pos:
      - -950
      - -486
    color:
      - -1
      - -1
      - -1
    z: 213
    minimized: false
    enabled: true
    flipped: false
  - state:
      params:
        max floor angle:
          type: angle
          name: max floor angle
          value: 0.3054326190990078
          min: -3.141592653589793
          max: 3.141592653589793
        max floor height:
          type: range
          name: max floor height
          double: -0.25
          min: -5
          max: 5
          step: 0.01
        min obstacle angle:
          value: 0.9032078879070662
          min: -3.141592653589793
          max: 3.141592653589793
          type: angle
          name: min obstacle angle
      persistent_params:
        []
    type: csapex::VelodyneSegmentation
    uuid: csapex::VelodyneSegmentation_0
    label: csapex::VelodyneSegmentation_0
    pos:
      - 79
      - 740
    color:
      - -1
      - -1
      - -1
    z: 741
    minimized: false
    enabled: true
    flipped: false
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_5
    label: csapex::ExportRos_5
    pos:
      - 537
      - 926
    color:
      - -1
      - -1
      - -1
    z: 676
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
  - state:
      persistent_params:
        []
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
          string: objects
    type: csapex::ExportRos
    uuid: csapex::ExportRos_6
    label: csapex::ExportRos_6
    pos:
      - 779
      - 173
    color:
      - -1
      - -1
      - -1
    z: 428
    minimized: false
    enabled: true
    flipped: false
  - type: csapex::ClusterPointcloud
    uuid: csapex::ClusterPointcloud_1
    label: csapex::ClusterPointcloud_1
    pos:
      - 466
      - 514
    color:
      - -1
      - -1
      - -1
    z: 684
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        ClusterTolerance:
          type: range
          name: ClusterTolerance
          double: 0.175
          min: 0.001
          max: 2
          step: 0.001
        MaxClusterSize:
          type: range
          name: MaxClusterSize
          int: 10000
          min: 0
          max: 100000
          step: 1
        MinClusterSize:
          type: range
          name: MinClusterSize
          int: 20
          min: 0
          max: 20000
          step: 1
        method:
          type: set
          name: method
          txt: POLAR
          int: 1
        opening_angle:
          type: angle
          name: opening_angle
          value: 0.1701696020694472
          min: -3.141592653589793
          max: 3.141592653589793
        radius:
          type: range
          name: radius
          double: 100
          min: 0
          max: 1000
          step: 0.1
      persistent_params:
        []
  - type: csapex::LabelClusteredPointCloud
    uuid: csapex::LabelClusteredPointCloud_0
    label: csapex::LabelClusteredPointCloud_0
    pos:
      - 359
      - 354
    color:
      - -1
      - -1
      - -1
    z: 430
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::ColorPointCloud
    uuid: csapex::ColorPointCloud_0
    label: csapex::ColorPointCloud_0
    pos:
      - 395
      - 242
    color:
      - -1
      - -1
      - -1
    z: 431
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_7
    label: csapex::ExportRos_7
    pos:
      - 1130
      - 331
    color:
      - -1
      - -1
      - -1
    z: 551
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
  - enabled: true
    flipped: false
    state:
      params:
        PCA:
          type: value
          name: PCA
          bool: true
      persistent_params:
        []
    type: csapex::ClustersToMarkerArray
    uuid: csapex::ClustersToMarkerArray_0
    label: csapex::ClustersToMarkerArray_0
    pos:
      - 923
      - 501
    color:
      - -1
      - -1
      - -1
    z: 686
    minimized: false
  - uuid: schoenbuch::LocalHazardField_0
    label: schoenbuch::LocalHazardField_0
    type: schoenbuch::LocalHazardField
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
    state:
      params:
        cell_resolution:
          type: range
          name: cell_resolution
          double: 1
          min: 0.1
          max: 10
          step: 0.1
        probability/decay:
          type: range
          name: probability/decay
          double: 0.24
          min: 0.01
          max: 0.99
          step: 0.01
        probability/decay_min_odds:
          type: range
          name: probability/decay_min_odds
          double: 0.26
          min: 0.01
          max: 0.99
          step: 0.01
        probability/decay_rate:
          type: range
          name: probability/decay_rate
          int: 2
          min: 0
          max: 100
          step: 1
        probability/free:
          type: range
          name: probability/free
          double: 0.01
          min: 0.01
          max: 0.99
          step: 0.01
        probability/measurement:
          type: range
          name: probability/measurement
          double: 0.68
          min: 0.01
          max: 0.99
          step: 0.01
        probability/threshold:
          type: range
          name: probability/threshold
          double: 0.92
          min: 0
          max: 1
          step: 0.01
        range:
          type: range
          name: range
          double: 10
          min: 0.2
          max: 10
          step: 0.1
        resolution:
          type: range
          name: resolution
          double: 0.09999999999999999
          min: 0.01
          max: 1
          step: 0.01
        size:
          type: range
          name: size
          double: 20
          min: 0.01
          max: 20
          step: 0.01
      persistent_params:
        []
  - type: csapex::DynamicTransform
    uuid: csapex::DynamicTransform_1
    label: csapex::DynamicTransform_1
    pos:
      - 863
      - 1106
    color:
      - -1
      - -1
      - -1
    z: 607
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        refresh:
          type: trigger
          name: refresh
        reset tf:
          type: trigger
          name: reset tf
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
      persistent_params:
        []
  - type: csapex::ExportRos
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
    enabled: true
    flipped: false
    state:
      params:
        queue:
          min: 1
          type: range
          name: queue
          int: 1
          max: 32
          step: 1
        topic:
          type: value
          name: topic
          string: /obstacle_cloud
      persistent_params:
        []
  - uuid: csapex::ExportRos_9
    pos:
      - 1668
      - 1174
    label: csapex::ExportRos_9
    color:
      - -1
      - -1
      - -1
    type: csapex::ExportRos
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
uuid_map:
  csapex::VelodyneUndistortion: 1
  schoenbuch::LocalHazardField: 1
  schoenbuch::PersonDetector: 1
  csapex::ColorPointCloud: 1
  csapex::DynamicTransform: 2
  csapex::ExportRos: 10
  csapex::ExtractTimeStamp: 2
  csapex::IMUIntegration: 1
  csapex::ImportRos: 5
  csapex::LabelClusteredPointCloud: 1
  csapex::LoamSegmentation: 1
  csapex::ObjectDetector: 1
  csapex::StaticTransform: 2
  csapex::TextDisplay: 16
  csapex::Note: 1
  csapex::Clock: 1
  csapex::CloudRenderer: 3
  csapex::ClustersToMarkerArray: 1
  csapex::ClusterPointcloud: 2
  csapex::TransformCloud: 1
  csapex::TransformInverter: 3
  csapex::TransformPublisher: 3
  csapex::VelodyneSegmentation: 1