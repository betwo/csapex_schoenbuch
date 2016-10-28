#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::ImportRos_0
    - uuid: csapex::IMUIntegration_0
      id: 1
    - uuid: csapex::ExtractMeasures_0
      id: 1
    - uuid: csapex::ImuToTransform_0
      id: 1
    - id: 1
      uuid: csapex::StaticTransform_0
    - uuid: csapex::TransformCombiner_0
      id: 1
    - uuid: csapex::TextDisplay_0
      id: 1
    - id: 1
      uuid: csapex::TimePlot_0
    - uuid: csapex::TimePlot_1
      id: 1
    - id: 1
      uuid: csapex::TimePlot_2
    - uuid: csapex::DynamicTransform_0
      id: 1
uuid_map:
  csapex::TimePlot: 3
  csapex::TransformCombiner: 1
  csapex::ImuToTransform: 1
  csapex::StaticTransform: 1
  csapex::TextDisplay: 1
  csapex::DynamicTransform: 1
  csapex::ExtractMeasures: 1
  csapex::IMUIntegration: 1
  csapex::ImportRos: 1
nodes:
  - z: 17
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    type: csapex::ImportRos
    uuid: csapex::ImportRos_0
    label: csapex::ImportRos_0
    pos:
      - -630
      - -261
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        buffer/max_wait:
          type: range
          name: buffer/max_wait
          double: 1
          min: 0
          max: 10
          step: 0.1
        buffer/length:
          step: 0.1
          type: range
          name: buffer/length
          double: 1
          min: 0
          max: 10
        latch:
          type: value
          name: latch
          bool: false
        queue:
          type: range
          name: queue
          int: 1
          min: 0
          max: 30
          step: 1
        rate:
          type: range
          name: rate
          double: 60
          min: 0.1
          max: 100
          step: 0.1
        refresh:
          type: trigger
          name: refresh
        topic:
          txt: /imu
          string: /imu
          type: set
          name: topic
  - type: csapex::ExtractMeasures
    uuid: csapex::ExtractMeasures_0
    label: csapex::ExtractMeasures_0
    pos:
      - 621
      - -95
    color:
      - -1
      - -1
      - -1
    z: 882
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      persistent_params:
        []
      params:
        dx:
          type: value
          name: dx
          double: -12.30454356997462
        dy:
          type: value
          name: dy
          double: -8.479618682035964
        dz:
          type: value
          name: dz
          double: -18.8569423433881
        norm:
          type: value
          name: norm
          double: 24.06013300046365
        pitch:
          type: angle
          name: pitch
          interactive: true
          value: 0.001050442615476548
          min: -3.141592653589793
          max: 3.141592653589793
        roll:
          type: angle
          name: roll
          interactive: true
          value: -0.01289954395118616
          min: -3.141592653589793
          max: 3.141592653589793
        yaw:
          type: angle
          name: yaw
          interactive: true
          value: 1.529515104664054
          min: -3.141592653589793
          max: 3.141592653589793
        yaw/cap:
          type: value
          name: yaw/cap
          bool: true
  - uuid: csapex::IMUIntegration_0
    label: csapex::IMUIntegration_0
    pos:
      - -21
      - -368
    color:
      - -1
      - -1
      - -1
    z: 10
    minimized: false
    type: csapex::IMUIntegration
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      persistent_params:
        []
      params:
        drift/yaw:
          type: range
          name: drift/yaw
          double: 0
          min: -1.1
          max: 1.1
          step: 0.0001
        offset/pitch:
          type: angle
          name: offset/pitch
          value: 0
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
  - uuid: csapex::ImuToTransform_0
    label: csapex::ImuToTransform_0
    pos:
      - -232
      - 110
    color:
      - -1
      - -1
      - -1
    type: csapex::ImuToTransform
    z: 18
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      persistent_params:
        []
      params:
        {}
  - uuid: csapex::StaticTransform_0
    type: csapex::StaticTransform
    label: csapex::StaticTransform_2
    pos:
      - 157
      - 441
    color:
      - -1
      - -1
      - -1
    z: 874
    minimized: false
    enabled: true
    flipped: true
    exec_mode: 1
    state:
      persistent_params:
        []
      params:
        child_frame:
          type: value
          name: child_frame
          string: /base_link
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
          string: /imu_link
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
  - uuid: csapex::TransformCombiner_0
    pos:
      - 175
      - 267
    label: csapex::TransformCombiner_0
    color:
      - -1
      - -1
      - -1
    z: 864
    type: csapex::TransformCombiner
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::TextDisplay_0
    type: csapex::TextDisplay
    label: csapex::TextDisplay_0
    pos:
      - 699
      - 418
    color:
      - -1
      - -1
      - -1
    z: 0
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::TimePlot
    uuid: csapex::TimePlot_0
    label: csapex::TimePlot_0
    pos:
      - 1184
      - -316
    color:
      - -1
      - -1
      - -1
    z: 877
    minimized: false
    enabled: true
    flipped: false
    exec_mode: 1
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
  - uuid: csapex::TimePlot_1
    pos:
      - 1193
      - 308
    color:
      - -1
      - -1
      - -1
    label: csapex::TimePlot_0
    minimized: false
    z: 878
    enabled: true
    type: csapex::TimePlot
    flipped: false
    exec_mode: 1
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
  - label: csapex::TimePlot_0
    uuid: csapex::TimePlot_2
    color:
      - -1
      - -1
      - -1
    pos:
      - 1205
      - 902
    minimized: false
    z: 883
    enabled: true
    type: csapex::TimePlot
    flipped: false
    exec_mode: 1
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
  - label: csapex::DynamicTransform_0
    uuid: csapex::DynamicTransform_0
    color:
      - -1
      - -1
      - -1
    pos:
      - -347
      - 463
    minimized: false
    z: 890
    enabled: true
    type: csapex::DynamicTransform
    flipped: false
    exec_mode: 1
    state:
      params:
        exact_time:
          type: value
          name: exact_time
          bool: false
        freeze_transformation:
          type: value
          name: freeze_transformation
          bool: false
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
          txt: /imu
          string: /imu
        ~tf/dx:
          type: value
          name: ~tf/dx
          double: 0
        ~tf/dy:
          type: value
          name: ~tf/dy
          double: 0
        ~tf/dz:
          type: value
          name: ~tf/dz
          double: 0
        ~tf/pitch:
          type: angle
          name: ~tf/pitch
          value: 0
          min: -3.141592653589793
          max: 3.141592653589793
        ~tf/roll:
          type: angle
          name: ~tf/roll
          value: 0
          min: -3.141592653589793
          max: 3.141592653589793
        ~tf/yaw:
          type: angle
          name: ~tf/yaw
          value: 0
          min: -3.141592653589793
          max: 3.141592653589793
      persistent_params:
        []
connections:
  - uuid: csapex::DynamicTransform_0:|:out_0
    targets:
      - csapex::TransformCombiner_0:|:in_1
    types:
      - default
  - uuid: csapex::ImportRos_0:|:out_0
    targets:
      - csapex::ImuToTransform_0:|:in_0
    types:
      - default
  - targets:
      - csapex::TransformCombiner_0:|:in_0
    types:
      - default
    uuid: csapex::ImuToTransform_0:|:out_0
  - uuid: csapex::TransformCombiner_0:|:out_0
    targets:
      - csapex::ExtractMeasures_0:|:in_0
      - csapex::TextDisplay_0:|:in_0
    types:
      - default
      - default
  - uuid: csapex::ExtractMeasures_0:|:out_pitch
    targets:
      - csapex::TimePlot_1:|:in_0
    types:
      - default
  - uuid: csapex::ExtractMeasures_0:|:out_roll
    targets:
      - csapex::TimePlot_0:|:in_0
    types:
      - default
  - uuid: csapex::ExtractMeasures_0:|:out_yaw
    targets:
      - csapex::TimePlot_2:|:in_0
    types:
      - default
adapters:
  []