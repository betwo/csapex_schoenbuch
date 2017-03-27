#!/home/buck/ws/apex/devel/lib/csapex/csapex_main
uuid_map:
  csapex::ExtractMeasures: 5
  csapex::ExtractTimeStamp: 2
  csapex::ParamEstimator: 1
  csapex::Clock: 1
  csapex::SignalLight: 1
  csapex::ExportRos: 4
  csapex::Graph: 2
  csapex::LAASImporter: 1
  csapex::MarkerMessageDetector: 1
  csapex::TextDisplay: 4
  csapex::TransformPublisher: 3
  csapex::TransformToOdometry: 3
  csapex::GenerateRelativeTransform: 1
adapters:
  []
connections:
  - uuid: event_activation
    targets:
      - csapex::SignalLight_0:|:slot_1
    types:
      - default
  - uuid: csapex::LAASImporter_0:|:out_3
    targets:
      - csapex::ExportRos_3:|:in_0
      - csapex::ParamEstimator_0:|:in_1
      - csapex::MarkerMessageDetector_0:|:in_0
    types:
      - default
      - default
      - default
  - uuid: csapex::LAASImporter_0:|:out_2
    targets:
      - csapex::ParamEstimator_0:|:in_0
    types:
      - default
  - uuid: csapex::MarkerMessageDetector_0:|:event_0
    targets:
      - csapex::SignalLight_0:|:slot_0
    types:
      - default
  - uuid: csapex::LAASImporter_0:|:event_0
    targets:
      - csapex::Graph_1:|:slot_2
      - csapex::Graph_0:|:slot_2
      - csapex::ParamEstimator_0:|:slot_0
      - csapex::SignalLight_0:|:slot_2
    types:
      - default
      - default
      - default
      - default
  - uuid: csapex::LAASImporter_0:|:out_1
    targets:
      - csapex::Graph_1:|:in_0
    types:
      - default
  - uuid: csapex::ExtractTimeStamp_0:|:out_0
    targets:
      - csapex::ExportRos_0:|:in_0
    types:
      - default
  - uuid: csapex::LAASImporter_0:|:out_0
    targets:
      - csapex::ExtractTimeStamp_0:|:in_0
      - csapex::Graph_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::ParamEstimator_0:|:out_0
    targets:
      - csapex::TransformToOdometry_2:|:in_0
    types:
      - default
  - uuid: csapex::TransformToOdometry_2:|:out_0
    targets:
      - csapex::ExportRos_2:|:in_0
    types:
      - default
threads:
  groups: ~
  assignments:
    - id: 1
      uuid: ~
    - id: 1
      uuid: csapex::LAASImporter_0
    - uuid: csapex::ExportRos_0
      id: 1
    - id: 1
      uuid: csapex::ExtractTimeStamp_0
    - uuid: csapex::Graph_0
      id: 1
    - id: 1
      uuid: csapex::Graph_0:|:csapex::ExportRos_0
    - id: 1
      uuid: csapex::Graph_0:|:csapex::TransformPublisher_0
    - id: 1
      uuid: csapex::Graph_0:|:csapex::GenerateRelativeTransform_0
    - uuid: csapex::Graph_0:|:csapex::TransformToOdometry_0
      id: 1
    - uuid: csapex::Graph_1
      id: 1
    - uuid: csapex::Graph_1:|:csapex::ExportRos_0
      id: 1
    - id: 1
      uuid: csapex::Graph_1:|:csapex::TransformPublisher_0
    - uuid: csapex::Graph_1:|:csapex::GenerateRelativeTransform_0
      id: 1
    - uuid: csapex::Graph_1:|:csapex::TransformToOdometry_0
      id: 1
    - id: 1
      uuid: csapex::ParamEstimator_0
    - uuid: csapex::ExportRos_2
      id: 1
    - id: 1
      uuid: csapex::TransformToOdometry_2
    - uuid: csapex::ExportRos_3
      id: 1
    - id: 1
      uuid: csapex::MarkerMessageDetector_0
    - uuid: csapex::SignalLight_0
      id: 1
nodes:
  - max_frequency: 0
    uuid: csapex::LAASImporter_0
    enabled: true
    logger_level: 1
    minimized: false
    label: csapex::LAASImporter_0
    muted: false
    flipped: false
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        wheels:
          value: /home/buck/data/schoenbuch/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/rmp440/rmp440_internal.log
          type: path
          name: wheels
        odometry:
          name: odometry
          value: /home/buck/data/schoenbuch/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/pom/pom.log
          type: path
        gps:
          name: gps
          value: /home/buck/data/schoenbuch/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/GPS/GPS.log
          type: path
        wheel_skip:
          int: 0
          name: wheel_skip
          type: value
        frame:
          name: frame
          min: 0
          type: range
          max: 5446
          int: 5446
          step: 1
        start:
          type: trigger
          name: start
    type: csapex::LAASImporter
    pos:
      - -554
      - -183
    exec_mode: 1
    z: 222
  - label: csapex::ExportRos_0
    flipped: false
    minimized: false
    exec_mode: 1
    color:
      - -1
      - -1
      - -1
    uuid: csapex::ExportRos_0
    pos:
      - 435
      - -391
    max_frequency: 0
    muted: false
    enabled: true
    type: csapex::ExportRos
    state:
      persistent_params:
        []
      params:
        queue:
          max: 32
          step: 1
          min: 1
          type: range
          name: queue
          int: 1
        topic:
          name: topic
          type: value
          string: /clock
    z: 22
    logger_level: 1
  - z: 31
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    minimized: false
    muted: false
    type: csapex::ExtractTimeStamp
    pos:
      - -51
      - -314
    color:
      - -1
      - -1
      - -1
    state:
      params:
        {}
      persistent_params:
        []
    uuid: csapex::ExtractTimeStamp_0
    max_frequency: 0
    label: csapex::ExtractTimeStamp_0
  - max_frequency: 0
    label: Publish GPS
    pos:
      - 20
      - -141
    exec_mode: 1
    logger_level: 1
    color:
      - -1
      - -1
      - -1
    z: 79
    state:
      params:
        output names:
          type: string_list
          name: output names
          list:
            []
        iterate_containers:
          type: value
          name: iterate_containers
          bool: false
        input names:
          type: string_list
          name: input names
          list:
            - Transform (absolute)
            - Transform (absolute)
        input count:
          name: input count
          int: 1
          type: value
        slot count:
          type: value
          name: slot count
          int: 1
        event names:
          type: string_list
          name: event names
          list:
            []
        event count:
          type: value
          name: event count
          int: 0
        iterated_containers:
          type: bitset
          name: iterated_containers
          int: 0
        output count:
          int: 0
          type: value
          name: output count
        slot names:
          list:
            - reset
            - reset
          type: string_list
          name: slot names
      persistent_params:
        []
    minimized: false
    subgraph:
      nodes:
        - color:
            - -1
            - -1
            - -1
          z: 43
          type: csapex::ExportRos
          enabled: true
          flipped: false
          exec_mode: 1
          logger_level: 1
          minimized: false
          muted: false
          state:
            persistent_params:
              []
            params:
              topic:
                type: value
                name: topic
                string: /odom_gps
              queue:
                step: 1
                name: queue
                int: 1
                min: 1
                max: 32
                type: range
          uuid: csapex::ExportRos_0
          max_frequency: 0
          label: csapex::ExportRos_1
          pos:
            - 851
            - 9
        - state:
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
          muted: false
          enabled: true
          flipped: false
          exec_mode: 1
          logger_level: 1
          label: csapex::TransformPublisher_2
          pos:
            - 708
            - -141
          color:
            - -1
            - -1
            - -1
          z: 32
          minimized: false
          type: csapex::TransformPublisher
          uuid: csapex::TransformPublisher_0
          max_frequency: 0
        - uuid: csapex::GenerateRelativeTransform_0
          label: csapex::GenerateRelativeTransform_0
          max_frequency: 0
          pos:
            - 20
            - -122
          logger_level: 1
          state:
            persistent_params:
              []
            params:
              {}
          type: csapex::GenerateRelativeTransform
          exec_mode: 1
          muted: false
          enabled: true
          flipped: false
          color:
            - -1
            - -1
            - -1
          minimized: false
          z: 45
        - uuid: csapex::TransformToOdometry_0
          max_frequency: 0
          label: csapex::TransformToOdometry_0
          pos:
            - 271
            - 17
          color:
            - -1
            - -1
            - -1
          z: 44
          minimized: false
          type: csapex::TransformToOdometry
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
      connections:
        - uuid: relayevent_0
          targets:
            - csapex::GenerateRelativeTransform_0:|:slot_0
          types:
            - default
        - uuid: relayout_0
          targets:
            - csapex::GenerateRelativeTransform_0:|:in_0
          types:
            - default
        - uuid: csapex::GenerateRelativeTransform_0:|:out_0
          targets:
            - csapex::TransformPublisher_0:|:in_0
            - csapex::TransformToOdometry_0:|:in_0
          types:
            - default
            - default
        - uuid: csapex::TransformToOdometry_0:|:out_0
          targets:
            - csapex::ExportRos_0:|:in_0
          types:
            - default
    type: csapex::Graph
    uuid: csapex::Graph_0
    muted: false
    enabled: true
    flipped: false
  - uuid: csapex::Graph_1
    max_frequency: 0
    label: Publish Odom
    pos:
      - 35
      - 5
    color:
      - -1
      - -1
      - -1
    z: 211
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    subgraph:
      connections:
        - uuid: relayevent_0
          targets:
            - csapex::GenerateRelativeTransform_0:|:slot_0
          types:
            - default
        - uuid: csapex::TransformToOdometry_0:|:out_0
          targets:
            - csapex::ExportRos_0:|:in_0
          types:
            - default
        - uuid: csapex::GenerateRelativeTransform_0:|:out_0
          targets:
            - csapex::TransformPublisher_0:|:in_0
            - csapex::TransformToOdometry_0:|:in_0
          types:
            - default
            - default
        - targets:
            - csapex::GenerateRelativeTransform_0:|:in_0
          types:
            - default
          uuid: relayout_0
      nodes:
        - flipped: false
          exec_mode: 1
          logger_level: 1
          z: 60
          minimized: false
          muted: false
          enabled: true
          type: csapex::ExportRos
          max_frequency: 0
          label: csapex::ExportRos_1
          pos:
            - 859
            - 47
          color:
            - -1
            - -1
            - -1
          uuid: csapex::ExportRos_0
          state:
            persistent_params:
              []
            params:
              topic:
                type: value
                name: topic
                string: /odom
              queue:
                name: queue
                int: 1
                min: 1
                max: 32
                step: 1
                type: range
        - state:
            persistent_params:
              []
            params:
              from:
                type: value
                name: from
                string: /
              to:
                string: /
                type: value
                name: to
          label: csapex::TransformPublisher_2
          pos:
            - 708
            - -141
          color:
            - -1
            - -1
            - -1
          z: 32
          type: csapex::TransformPublisher
          uuid: csapex::TransformPublisher_0
          max_frequency: 0
          minimized: false
          muted: false
          enabled: true
          flipped: false
          exec_mode: 1
          logger_level: 1
        - state:
            persistent_params:
              []
            params:
              {}
          type: csapex::GenerateRelativeTransform
          max_frequency: 0
          label: csapex::GenerateRelativeTransform_0
          uuid: csapex::GenerateRelativeTransform_0
          pos:
            - 20
            - -122
          color:
            - -1
            - -1
            - -1
          z: 45
          minimized: false
          muted: false
          enabled: true
          flipped: false
          exec_mode: 1
          logger_level: 1
        - type: csapex::TransformToOdometry
          uuid: csapex::TransformToOdometry_0
          max_frequency: 0
          label: csapex::TransformToOdometry_0
          pos:
            - 271
            - 17
          color:
            - -1
            - -1
            - -1
          z: 44
          state:
            persistent_params:
              []
            params:
              {}
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
        output names:
          type: string_list
          name: output names
          list:
            []
        slot count:
          type: value
          name: slot count
          int: 1
        event count:
          type: value
          name: event count
          int: 0
        event names:
          type: string_list
          name: event names
          list:
            []
        iterate_containers:
          type: value
          name: iterate_containers
          bool: false
        input count:
          type: value
          name: input count
          int: 1
        input names:
          name: input names
          list:
            - Transform (absolute)
            - Transform (absolute)
          type: string_list
        slot names:
          type: string_list
          name: slot names
          list:
            - reset
            - reset
        iterated_containers:
          type: bitset
          name: iterated_containers
          int: 0
        output count:
          type: value
          name: output count
          int: 0
    type: csapex::Graph
  - state:
      persistent_params:
        []
      params:
        y_icr_r:
          type: range
          name: y_icr_r
          double: -0.75
          min: -0.75
          max: -0.001
          step: 0.001
        alpha_r:
          type: range
          name: alpha_r
          double: 1
          min: 0.5
          max: 1
          step: 0.001
        reset:
          type: trigger
          name: reset
        alpha_l:
          step: 0.001
          type: range
          name: alpha_l
          double: 1
          min: 0.5
          max: 1
        x_icr:
          type: range
          name: x_icr
          double: -0.1
          min: -0.1
          max: 0
          step: 0.001
        y_icr_l:
          type: range
          name: y_icr_l
          double: 0.75
          min: 0.001
          max: 0.75
          step: 0.001
    uuid: csapex::ParamEstimator_0
    max_frequency: 0
    label: csapex::ParamEstimator_0
    pos:
      - -32
      - 218
    color:
      - -1
      - -1
      - -1
    logger_level: 1
    type: csapex::ParamEstimator
    z: 206
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_2
    max_frequency: 0
    pos:
      - 531
      - 341
    label: csapex::ExportRos_1
    color:
      - -1
      - -1
      - -1
    z: 217
    minimized: false
    muted: false
    enabled: true
    flipped: false
    state:
      persistent_params:
        []
      params:
        topic:
          type: value
          name: topic
          string: /odom_est
        queue:
          step: 1
          type: range
          name: queue
          int: 1
          min: 1
          max: 32
    exec_mode: 1
    logger_level: 1
  - flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        {}
    type: csapex::TransformToOdometry
    uuid: csapex::TransformToOdometry_2
    max_frequency: 0
    label: csapex::TransformToOdometry_2
    pos:
      - 454
      - 518
    color:
      - -1
      - -1
      - -1
    z: 109
    minimized: false
    muted: false
    enabled: true
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_3
    max_frequency: 0
    state:
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
          string: export
    flipped: false
    exec_mode: 1
    logger_level: 1
    label: csapex::ExportRos_3
    pos:
      - -436
      - 447
    color:
      - -1
      - -1
      - -1
    z: 203
    minimized: false
    muted: false
    enabled: true
  - flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      persistent_params:
        []
      params:
        type:
          type: bitset
          name: type
          int: 2
    type: csapex::MarkerMessageDetector
    uuid: csapex::MarkerMessageDetector_0
    max_frequency: 0
    label: csapex::MarkerMessageDetector_0
    pos:
      - 480
      - -192
    color:
      - -1
      - -1
      - -1
    z: 216
    minimized: false
    muted: false
    enabled: true
  - color:
      - -1
      - -1
      - -1
    z: 223
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
        state:
          type: set
          name: state
          txt: red
          int: 2
    type: csapex::SignalLight
    uuid: csapex::SignalLight_0
    max_frequency: 0
    label: csapex::SignalLight_0
    pos:
      - 625
      - 88