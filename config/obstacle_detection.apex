#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
uuid_map:
  csapex::ExtractTimeStamp: 2
  csapex::IMUIntegration: 1
  csapex::ImportRos: 5
  csapex::ExportRos: 10
  csapex::StaticTransform: 2
  csapex::TextDisplay: 15
  csapex::TransformCloud: 1
  csapex::TransformInverter: 3
  csapex::TransformPublisher: 3
  csapex::VelodyneSegmentation: 1
  csapex::Clock: 1
  csapex::LabelClusteredPointCloud: 1
  csapex::LoamSegmentation: 1
  csapex::Note: 1
  csapex::ObjectDetector: 1
  csapex::CloudRenderer: 3
  csapex::ClusterPointcloud: 2
  csapex::ClustersToMarkerArray: 1
  csapex::ColorPointCloud: 1
  csapex::DynamicTransform: 2
  csapex::VelodyneUndistortion: 1
  schoenbuch::LocalHazardField: 1
  schoenbuch::PersonDetector: 1
threads:
  assignments:
    - id: 1
      uuid: ~
    - uuid: csapex::ImportRos_3
      id: 1
    - uuid: csapex::VelodyneUndistortion_0
      id: 1
    - id: 1
      uuid: csapex::ExportRos_0
    - uuid: csapex::VelodyneSegmentation_0
      id: 0
    - uuid: schoenbuch::LocalHazardField_0
      id: 0
    - id: 1
      uuid: csapex::DynamicTransform_1
    - uuid: csapex::ExportRos_8
      id: 1
  groups:
    - id: 2
      name: Imu Tf
nodes:
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
      persistent_params:
        []
  - pos:
      - -335
      - 921
    label: csapex::VelodyneUndistortion_0
    uuid: csapex::VelodyneUndistortion_0
    color:
      - -1
      - -1
      - -1
    z: 483
    minimized: false
    type: csapex::VelodyneUndistortion
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
  - uuid: csapex::ExportRos_0
    label: csapex::ExportRos_0
    pos:
      - 71
      - 1050
    type: csapex::ExportRos
    color:
      - -1
      - -1
      - -1
    z: 693
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
  - label: csapex::VelodyneSegmentation_0
    uuid: csapex::VelodyneSegmentation_0
    pos:
      - 79
      - 740
    type: csapex::VelodyneSegmentation
    color:
      - -1
      - -1
      - -1
    z: 689
    minimized: false
    enabled: true
    flipped: false
    state:
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
          type: angle
          name: min obstacle angle
          value: 0.9032078879070662
          min: -3.141592653589793
          max: 3.141592653589793
      persistent_params:
        []
  - uuid: schoenbuch::LocalHazardField_0
    pos:
      - 787
      - 522
    label: schoenbuch::LocalHazardField_0
    color:
      - -1
      - -1
      - -1
    type: schoenbuch::LocalHazardField
    z: 691
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
          double: 0.3
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
  - uuid: csapex::DynamicTransform_1
    pos:
      - 404
      - 1135
    label: csapex::DynamicTransform_1
    enabled: true
    flipped: false
    z: 692
    color:
      - -1
      - -1
      - -1
    minimized: false
    type: csapex::DynamicTransform
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
  - uuid: csapex::ExportRos_8
    label: csapex::ExportRos_8
    pos:
      - 1403
      - 662
    type: csapex::ExportRos
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
          string: /obstacle_cloud
      persistent_params:
        []
connections:
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
  - uuid: csapex::VelodyneSegmentation_0:|:out_0
    targets:
      - schoenbuch::LocalHazardField_0:|:in_0
    types:
      - default
  - uuid: schoenbuch::LocalHazardField_0:|:out_0
    targets:
      - csapex::ExportRos_8:|:in_0
    types:
      - default
  - uuid: csapex::DynamicTransform_1:|:out_0
    targets:
      - schoenbuch::LocalHazardField_0:|:in_1
    types:
      - default
adapters:
  []