#!/home/huskic/workspace/apex/devel/lib/csapex/csapex_main
uuid_map:
  csapex::TransformPublisher: 3
  csapex::TransformToOdometry: 1
  csapex::TextDisplay: 1
  csapex::ExportRos: 2
  csapex::ExtractMeasures: 1
  csapex::ExtractTimeStamp: 1
  csapex::GenerateRelativeTransform: 1
  csapex::Graph: 2
  csapex::LAASImporter: 1
  csapex::Clock: 1
threads:
  groups: ~
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::LAASImporter_0
    - uuid: csapex::ExportRos_0
      id: 1
    - uuid: csapex::ExtractTimeStamp_0
      id: 1
    - uuid: csapex::Graph_0
      id: 1
    - uuid: csapex::Graph_0:|:csapex::ExportRos_0
      id: 1
    - uuid: csapex::Graph_0:|:csapex::TransformPublisher_0
      id: 1
    - id: 1
      uuid: csapex::Graph_0:|:csapex::GenerateRelativeTransform_0
    - uuid: csapex::Graph_0:|:csapex::TransformToOdometry_0
      id: 1
    - uuid: csapex::Graph_1
      id: 1
    - uuid: csapex::Graph_1:|:csapex::ExportRos_0
      id: 1
    - uuid: csapex::Graph_1:|:csapex::TransformPublisher_0
      id: 1
    - uuid: csapex::Graph_1:|:csapex::GenerateRelativeTransform_0
      id: 1
    - id: 1
      uuid: csapex::Graph_1:|:csapex::TransformToOdometry_0
nodes:
  - label: csapex::LAASImporter_0
    pos:
      - -544
      - -176
    color:
      - -1
      - -1
      - -1
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    type: csapex::LAASImporter
    uuid: csapex::LAASImporter_0
    max_frequency: 0
    z: 80
    minimized: false
    state:
      params:
        gps:
          type: path
          name: gps
          value: /home/huskic/Documents/LAAS-CNRS/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/GPS/GPS.log
        odometry:
          type: path
          name: odometry
          value: /home/huskic/Documents/LAAS-CNRS/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/pom/pom.log
        start:
          type: trigger
          name: start
        wheels:
          type: path
          name: wheels
          value: /home/huskic/Documents/LAAS-CNRS/minnieLogs/log_2016_12_13_12_19_11-onGrass/log_16-12-13_12-20-07/rmp440/rmp440_internal.log
      persistent_params:
        []
  - max_frequency: 0
    uuid: csapex::ExportRos_0
    label: csapex::ExportRos_0
    pos:
      - 435
      - -391
    type: csapex::ExportRos
    color:
      - -1
      - -1
      - -1
    z: 22
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
          string: /clock
      persistent_params:
        []
  - uuid: csapex::ExtractTimeStamp_0
    max_frequency: 0
    label: csapex::ExtractTimeStamp_0
    type: csapex::ExtractTimeStamp
    pos:
      - -51
      - -314
    color:
      - -1
      - -1
      - -1
    z: 31
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
  - uuid: csapex::Graph_0
    max_frequency: 0
    type: csapex::Graph
    label: Publish GPS
    pos:
      - 20
      - -141
    color:
      - -1
      - -1
      - -1
    z: 79
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    state:
      params:
        event count:
          type: value
          name: event count
          int: 0
        event names:
          type: string_list
          name: event names
          list:
            []
        input count:
          type: value
          name: input count
          int: 1
        input names:
          type: string_list
          name: input names
          list:
            - Transform (absolute)
            - Transform (absolute)
        iterate_containers:
          type: value
          name: iterate_containers
          bool: false
        iterated_containers:
          type: bitset
          name: iterated_containers
          int: 0
        output count:
          type: value
          name: output count
          int: 0
        output names:
          type: string_list
          name: output names
          list:
            []
        slot count:
          type: value
          name: slot count
          int: 1
        slot names:
          type: string_list
          name: slot names
          list:
            - reset
            - reset
      persistent_params:
        []
    subgraph:
      nodes:
        - uuid: csapex::ExportRos_0
          max_frequency: 0
          label: csapex::ExportRos_1
          pos:
            - 851
            - 9
          color:
            - -1
            - -1
            - -1
          type: csapex::ExportRos
          z: 43
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
                string: /odom_gps
            persistent_params:
              []
        - uuid: csapex::TransformPublisher_0
          max_frequency: 0
          label: csapex::TransformPublisher_2
          pos:
            - 708
            - -141
          type: csapex::TransformPublisher
          color:
            - -1
            - -1
            - -1
          z: 32
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
        - type: csapex::GenerateRelativeTransform
          uuid: csapex::GenerateRelativeTransform_0
          max_frequency: 0
          label: csapex::GenerateRelativeTransform_0
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
          state:
            params:
              {}
            persistent_params:
              []
        - uuid: csapex::TransformToOdometry_0
          max_frequency: 0
          type: csapex::TransformToOdometry
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
        - targets:
            - csapex::ExportRos_0:|:in_0
          uuid: csapex::TransformToOdometry_0:|:out_0
          types:
            - default
        - uuid: csapex::GenerateRelativeTransform_0:|:out_0
          targets:
            - csapex::TransformPublisher_0:|:in_0
            - csapex::TransformToOdometry_0:|:in_0
          types:
            - default
            - default
        - uuid: relayout_0
          targets:
            - csapex::GenerateRelativeTransform_0:|:in_0
          types:
            - default
        - uuid: relayevent_0
          targets:
            - csapex::GenerateRelativeTransform_0:|:slot_0
          types:
            - default
  - uuid: csapex::Graph_1
    pos:
      - 35
      - 5
    color:
      - -1
      - -1
      - -1
    label: Publish Odom
    max_frequency: 0
    z: 76
    minimized: false
    muted: false
    enabled: true
    flipped: false
    exec_mode: 1
    logger_level: 1
    type: csapex::Graph
    state:
      params:
        event count:
          type: value
          name: event count
          int: 0
        event names:
          type: string_list
          name: event names
          list:
            []
        input count:
          type: value
          name: input count
          int: 1
        input names:
          type: string_list
          name: input names
          list:
            - Transform (absolute)
            - Transform (absolute)
        iterate_containers:
          type: value
          name: iterate_containers
          bool: false
        iterated_containers:
          type: bitset
          name: iterated_containers
          int: 0
        output count:
          type: value
          name: output count
          int: 0
        output names:
          type: string_list
          name: output names
          list:
            []
        slot count:
          type: value
          name: slot count
          int: 1
        slot names:
          type: string_list
          name: slot names
          list:
            - reset
            - reset
      persistent_params:
        []
    subgraph:
      nodes:
        - uuid: csapex::ExportRos_0
          max_frequency: 0
          label: csapex::ExportRos_1
          pos:
            - 859
            - 47
          color:
            - -1
            - -1
            - -1
          type: csapex::ExportRos
          z: 60
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
        - uuid: csapex::TransformPublisher_0
          max_frequency: 0
          label: csapex::TransformPublisher_2
          pos:
            - 708
            - -141
          type: csapex::TransformPublisher
          color:
            - -1
            - -1
            - -1
          z: 32
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
        - type: csapex::GenerateRelativeTransform
          uuid: csapex::GenerateRelativeTransform_0
          max_frequency: 0
          label: csapex::GenerateRelativeTransform_0
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
          state:
            params:
              {}
            persistent_params:
              []
        - uuid: csapex::TransformToOdometry_0
          max_frequency: 0
          type: csapex::TransformToOdometry
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
        - uuid: relayevent_0
          targets:
            - csapex::GenerateRelativeTransform_0:|:slot_0
          types:
            - default
adapters:
  []
connections:
  - targets:
      - csapex::ExtractTimeStamp_0:|:in_0
      - csapex::Graph_0:|:in_0
    types:
      - default
      - default
    uuid: csapex::LAASImporter_0:|:out_0
  - uuid: csapex::ExtractTimeStamp_0:|:out_0
    targets:
      - csapex::ExportRos_0:|:in_0
    types:
      - default
  - uuid: csapex::LAASImporter_0:|:out_1
    targets:
      - csapex::Graph_1:|:in_0
    types:
      - default
  - uuid: csapex::LAASImporter_0:|:event_0
    targets:
      - csapex::Graph_1:|:slot_2
      - csapex::Graph_0:|:slot_2
    types:
      - default
      - default