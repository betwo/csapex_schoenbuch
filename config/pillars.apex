#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - uuid: csapex::ImportRos_0
      id: 1
    - uuid: csapex::PointCloudToIntensityImage_0
      id: 1
    - uuid: csapex::OutputDisplay_0
      id: 1
    - id: 1
      uuid: csapex::CloudRenderer_0
    - id: 1
      uuid: vision_plugins::GenericImageCombiner_0
    - id: 1
      uuid: csapex::ObjectDetector_0
    - id: 1
      uuid: csapex::OutputDisplay_1
    - id: 1
      uuid: csapex::OutputDisplay_4
    - uuid: csapex::Graph_0
      id: 1
    - id: 1
      uuid: csapex::PointCloudToColorImage_0
    - id: 1
      uuid: vision_plugins::ColorConvert_0
    - id: 1
      uuid: vision_plugins::Resize_0
    - id: 1
      uuid: csapex::PointCloudToIntensityImage_0
    - uuid: vision_plugins::Resize_1
      id: 1
    - id: 1
      uuid: vision_plugins::Resize_2
    - uuid: csapex::PointCloudToDepthImage_0
      id: 1
    - id: 1
      uuid: vision_plugins::ColorConvert_1
    - uuid: csapex::OutputDisplay_0
      id: 1
    - id: 1
      uuid: csapex::PassThrough_0
    - id: 1
      uuid: csapex::PassThrough_1
    - uuid: csapex::OutputDisplay_5
      id: 1
    - uuid: csapex::ExportRos_0
      id: 1
uuid_map:
  csapex::Graph: 1
  vision_plugins::FloatToUchar: 1
  patsy::ObstacleDetector: 1
  csapex::PointCloudToIntensityImage: 2
  csapex::RoiGenerator: 1
  vision_plugins::ColorConvert: 1
  csapex::PillarLocalization: 1
  csapex::PointCloudToColorImage: 1
  csapex::PointCloudToDepthImage: 1
  csapex::CloudRenderer: 2
  vision_plugins::GenericImageCombiner: 1
  vision_plugins::Scale: 1
  vision_plugins::Resize: 3
  csapex::ImportRos: 1
  csapex::ObjectDetector: 1
  csapex::OutputDisplay: 6
  csapex::ImageRoi: 1
  csapex::ExportRos: 1
nodes:
  - state:
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
        rate:
          type: range
          name: rate
          double: 10
          max: 100
          min: 0.1
          step: 0.1
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
    pos:
      - -936
      - -122
    color:
      - -1
      - -1
      - -1
    type: csapex::ImportRos
    uuid: csapex::ImportRos_0
    label: csapex::ImportRos_0
    z: 582
    minimized: false
    enabled: true
    flipped: false
  - type: csapex::PointCloudToIntensityImage
    uuid: csapex::PointCloudToIntensityImage_0
    label: csapex::PointCloudToIntensityImage_0
    pos:
      - -493
      - -421
    color:
      - -1
      - -1
      - -1
    z: 16
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - color:
      - -1
      - -1
      - -1
    z: 22
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
    type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_0
    label: csapex::OutputDisplay_0
    pos:
      - -78
      - -605
  - state:
      persistent_params:
        []
      params:
        color/gradient/start:
          type: color
          name: color/gradient/start
          values:
            - 0
            - 255
            - 0
        color/grid:
          type: color
          name: color/grid
          values:
            - 0
            - 0
            - 0
        color/rainbow:
          type: value
          name: color/rainbow
          bool: false
        point/size:
          type: range
          name: point/size
          double: 9.300000000000001
          min: 1
          max: 30
          step: 0.1
        show axes:
          type: value
          name: show axes
          bool: false
        ~grid/resolution:
          type: range
          name: ~grid/resolution
          double: 1
          min: 0.1
          max: 10
          step: 0.1
        ~view/dy:
          type: range
          name: ~view/dy
          double: -4.475581169128418
          min: -10
          max: 10
          step: 0.01
        ~view/dz:
          type: range
          name: ~view/dz
          double: 0
          min: -10
          max: 10
          step: 0.01
        ~view/phi:
          type: range
          name: ~view/phi
          double: -0.5204440784612236
          min: -3.141592653589793
          max: 3.141592653589793
          step: 0.001
        ~view/r:
          type: range
          name: ~view/r
          double: 13.59999999999998
          min: 0.01
          max: 20
          step: 0.01
        ~view/theta:
          type: range
          name: ~view/theta
          double: 2.150592653589788
          min: 0
          max: 3.141592653589793
          step: 0.001
        ~size/out/height:
          type: range
          name: ~size/out/height
          int: 763
          min: 10
          max: 1024
          step: 1
        ~size/out/sync:
          type: value
          name: ~size/out/sync
          bool: true
        ~size/out/width:
          type: range
          name: ~size/out/width
          int: 921
          min: 10
          max: 1024
          step: 1
        ~size/width:
          type: range
          name: ~size/width
          int: 921
          min: 10
          max: 1024
          step: 1
        ~view/dx:
          type: range
          name: ~view/dx
          double: 6.962682247161865
          min: -10
          max: 10
          step: 0.01
        ~grid/size:
          type: range
          name: ~grid/size
          int: 10
          min: 1
          max: 30
          step: 1
        ~grid/xy:
          type: value
          name: ~grid/xy
          bool: true
        ~grid/xz:
          type: value
          name: ~grid/xz
          bool: false
        ~grid/yz:
          type: value
          name: ~grid/yz
          bool: false
        ~size/height:
          step: 1
          type: range
          name: ~size/height
          int: 763
          min: 10
          max: 1024
        color/background:
          type: color
          name: color/background
          values:
            - 255
            - 255
            - 255
        color/field:
          type: set
          name: color/field
          txt: rgb
          string: rgb
        color/force gradient:
          type: value
          name: color/force gradient
          bool: false
        color/gradient/end:
          type: color
          name: color/gradient/end
          values:
            - 0
            - 0
            - 255
    type: csapex::CloudRenderer
    uuid: csapex::CloudRenderer_0
    label: csapex::CloudRenderer_0
    pos:
      - 1347
      - -573
    color:
      - -1
      - -1
      - -1
    z: 562
    minimized: false
    enabled: true
    flipped: false
  - state:
      persistent_params:
        []
      params:
        curvature/threshold:
          type: range
          name: curvature/threshold
          double: 2.99
          min: 0
          max: 4
          step: 0.01
        pillar/min intensity:
          type: range
          name: pillar/min intensity
          double: 138.1
          min: 0
          max: 1024
          step: 0.1
        pillar/min points:
          type: range
          name: pillar/min points
          int: 7
          min: 0
          max: 512
          step: 1
        pillar/radius:
          type: range
          name: pillar/radius
          double: 0.065
          min: 0.001
          max: 1
          step: 0.001
        pillar/radius threshold:
          type: range
          name: pillar/radius threshold
          double: 0.05
          min: 0
          max: 1
          step: 0.001
        vertical/angle threshold:
          type: angle
          name: vertical/angle threshold
          value: 1.50534647984511
          min: -3.141592653589793
          max: 3.141592653589793
        clustering/max cluster diameter:
          type: range
          name: clustering/max cluster diameter
          double: 0
          min: 0
          max: 5
          step: 0.01
        clustering/max cluster size:
          type: range
          name: clustering/max cluster size
          int: 37
          min: 0
          max: 1024
          step: 1
        clustering/max distance on ring:
          type: range
          name: clustering/max distance on ring
          double: 0.043
          min: 0
          max: 0.1
          step: 0.001
        clustering/max distance vertically:
          type: range
          name: clustering/max distance vertically
          double: 0.026
          min: 0
          max: 0.1
          step: 0.001
        clustering/min cluster size:
          type: range
          name: clustering/min cluster size
          int: 2
          min: 0
          max: 1024
          step: 1
        curvature threshold:
          type: range
          name: curvature threshold
          double: 1.81
          min: 0
          max: 4
          step: 0.01
        curvature/min area:
          type: range
          name: curvature/min area
          double: 0.12
          min: 0
          step: 0.01
          max: 1
    type: csapex::ObjectDetector
    uuid: csapex::ObjectDetector_0
    label: csapex::ObjectDetector_0
    pos:
      - 24
      - 167
    color:
      - -1
      - -1
      - -1
    z: 581
    minimized: false
    enabled: true
    flipped: false
  - type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_1
    label: csapex::OutputDisplay_1
    pos:
      - 865
      - 762
    color:
      - -1
      - -1
      - -1
    z: 468
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: vision_plugins::GenericImageCombiner
    uuid: vision_plugins::GenericImageCombiner_0
    label: vision_plugins::GenericImageCombiner_0
    pos:
      - 336
      - 1230
    color:
      - -1
      - -1
      - -1
    z: 398
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        script:
          type: value
          name: script
          string: $1
      persistent_params:
        []
  - type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_4
    label: csapex::OutputDisplay_4
    pos:
      - 857
      - 1384
    color:
      - -1
      - -1
      - -1
    z: 453
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::Graph
    uuid: csapex::Graph_0
    label: Convert to images
    pos:
      - 45
      - 975
    color:
      - -1
      - -1
      - -1
    z: 403
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
    subgraph:
      nodes:
        - type: csapex::PointCloudToColorImage
          uuid: csapex::PointCloudToColorImage_0
          label: csapex::PointCloudToColorImage_0
          pos:
            - 136
            - 869
          color:
            - -1
            - -1
            - -1
          z: 351
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              {}
            persistent_params:
              []
        - label: vision_plugins::ColorConvert_0
          uuid: vision_plugins::ColorConvert_0
          pos:
            - 621
            - 1232
          type: vision_plugins::ColorConvert
          color:
            - -1
            - -1
            - -1
          z: 386
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              input:
                type: set
                name: input
                txt: MONO
                int: 5
              output:
                type: set
                name: output
                txt: BGR
                int: 2
            persistent_params:
              []
        - uuid: vision_plugins::Resize_0
          label: vision_plugins::Resize_0
          pos:
            - 572
            - 878
          type: vision_plugins::Resize
          color:
            - -1
            - -1
            - -1
          z: 307
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              mode:
                type: set
                name: mode
                txt: nearest
                int: 0
              size height:
                type: range
                name: size height
                interactive: true
                int: 480
                min: 1
                max: 10000
                step: 1
              size width:
                type: range
                name: size width
                interactive: true
                int: 2048
                min: 1
                max: 10000
                step: 1
            persistent_params:
              []
        - type: csapex::PointCloudToIntensityImage
          uuid: csapex::PointCloudToIntensityImage_0
          label: csapex::PointCloudToIntensityImage_1
          pos:
            - -160
            - 1143
          color:
            - -1
            - -1
            - -1
          z: 348
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              {}
            persistent_params:
              []
        - uuid: vision_plugins::Resize_1
          label: vision_plugins::Resize_0
          type: vision_plugins::Resize
          pos:
            - 265
            - 1063
          color:
            - -1
            - -1
            - -1
          z: 388
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              mode:
                type: set
                name: mode
                txt: nearest
                int: 0
              size height:
                type: range
                name: size height
                interactive: true
                int: 480
                min: 1
                max: 10000
                step: 1
              size width:
                type: range
                name: size width
                interactive: true
                int: 2048
                min: 1
                max: 10000
                step: 1
            persistent_params:
              []
        - type: vision_plugins::Resize
          uuid: vision_plugins::Resize_2
          label: vision_plugins::Resize_0
          pos:
            - 297
            - 1332
          color:
            - -1
            - -1
            - -1
          z: 349
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              mode:
                type: set
                name: mode
                txt: nearest
                int: 0
              size height:
                type: range
                name: size height
                interactive: true
                int: 480
                min: 1
                max: 10000
                step: 1
              size width:
                type: range
                name: size width
                interactive: true
                int: 2048
                min: 1
                max: 10000
                step: 1
            persistent_params:
              []
        - type: csapex::PointCloudToDepthImage
          uuid: csapex::PointCloudToDepthImage_0
          label: csapex::PointCloudToDepthImage_0
          pos:
            - -67
            - 967
          color:
            - -1
            - -1
            - -1
          z: 432
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              fit:
                type: value
                name: fit
                bool: true
              scale:
                type: range
                name: scale
                double: 2.5
                min: 1
                max: 1000
                step: 0.5
            persistent_params:
              []
        - uuid: vision_plugins::ColorConvert_1
          label: vision_plugins::ColorConvert_0
          pos:
            - 877
            - 1072
          color:
            - -1
            - -1
            - -1
          type: vision_plugins::ColorConvert
          z: 389
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              input:
                type: set
                name: input
                txt: MONO
                int: 5
              output:
                type: set
                name: output
                txt: BGR
                int: 2
            persistent_params:
              []
        - uuid: csapex::OutputDisplay_0
          label: csapex::OutputDisplay_0
          pos:
            - 842
            - 1546
          type: csapex::OutputDisplay
          color:
            - -1
            - -1
            - -1
          z: 413
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              {}
            persistent_params:
              []
        - uuid: csapex::PassThrough_0
          label: csapex::PassThrough_0
          type: csapex::PassThrough
          pos:
            - -610
            - 812
          color:
            - -1
            - -1
            - -1
          z: 427
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              field:
                type: set
                name: field
                txt: x
                string: x
              interval:
                type: interval
                name: interval
                double:
                  - -10
                  - 10
                min: -10
                max: 10
                step: 0.01
              keep organized:
                type: value
                name: keep organized
                bool: true
            persistent_params:
              []
        - uuid: csapex::PassThrough_1
          label: csapex::PassThrough_1
          pos:
            - -569
            - 1005
          type: csapex::PassThrough
          color:
            - -1
            - -1
            - -1
          z: 429
          minimized: false
          enabled: true
          flipped: false
          state:
            params:
              field:
                type: set
                name: field
                txt: y
                string: y
              interval:
                type: interval
                name: interval
                double:
                  - -10
                  - 10
                min: -10
                max: 10
                step: 0.01
              keep organized:
                type: value
                name: keep organized
                bool: true
            persistent_params:
              []
      forward_in:
        - uuid_external: csapex::Graph_0:|:in_1
          uuid_internal: relayout_1
          type: PointCloud
          optional: false
          label: forwarding
        - uuid_external: csapex::Graph_0:|:in_2
          uuid_internal: relayout_2
          type: PointCloud
          optional: false
          label: forwarding
        - uuid_external: csapex::Graph_0:|:in_0
          uuid_internal: relayout_0
          type: PointCloud
          optional: false
          label: forwarding
      forward_out:
        - uuid_external: csapex::Graph_0:|:out_0
          uuid_internal: relayin_0
          type: cv::Mat
          label: forwarding
        - uuid_external: csapex::Graph_0:|:out_1
          uuid_internal: relayin_1
          type: cv::Mat
          label: forwarding
        - uuid_external: csapex::Graph_0:|:out_2
          uuid_internal: relayin_2
          type: cv::Mat
          label: forwarding
      connections:
        - uuid: relayout_2
          targets:
            - csapex::PassThrough_0:|:in_0
        - uuid: relayout_1
          targets:
            - csapex::PointCloudToIntensityImage_0:|:in_0
        - uuid: vision_plugins::Resize_1:|:out_size height
          targets:
            - vision_plugins::Resize_0:|:in_size height
            - vision_plugins::Resize_2:|:in_size height
        - uuid: vision_plugins::Resize_1:|:out_size width
          targets:
            - vision_plugins::Resize_0:|:in_size width
            - vision_plugins::Resize_2:|:in_size width
        - uuid: relayout_0
          targets:
            - csapex::PointCloudToColorImage_0:|:in_0
        - uuid: vision_plugins::Resize_0:|:out_0
          targets:
            - relayin_1
        - uuid: csapex::PointCloudToDepthImage_0:|:out_0
          targets:
            - vision_plugins::Resize_1:|:in_0
        - uuid: vision_plugins::Resize_2:|:out_0
          targets:
            - vision_plugins::ColorConvert_0:|:in_0
        - uuid: csapex::PointCloudToIntensityImage_0:|:out_0
          targets:
            - vision_plugins::Resize_2:|:in_0
        - uuid: csapex::PointCloudToColorImage_0:|:out_0
          targets:
            - vision_plugins::Resize_0:|:in_0
        - uuid: vision_plugins::Resize_1:|:out_0
          targets:
            - vision_plugins::ColorConvert_1:|:in_0
        - uuid: vision_plugins::ColorConvert_1:|:out_0
          targets:
            - relayin_2
            - csapex::OutputDisplay_0:|:in_0
        - uuid: vision_plugins::ColorConvert_0:|:out_0
          targets:
            - relayin_0
        - uuid: csapex::PassThrough_0:|:out_0
          targets:
            - csapex::PassThrough_1:|:in_0
        - uuid: csapex::PassThrough_1:|:out_0
          targets:
            - csapex::PointCloudToDepthImage_0:|:in_0
  - label: csapex::OutputDisplay_5
    pos:
      - 862
      - 1976
    color:
      - -1
      - -1
      - -1
    uuid: csapex::OutputDisplay_5
    z: 454
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
    type: csapex::OutputDisplay
  - type: csapex::ExportRos
    uuid: csapex::ExportRos_0
    label: csapex::ExportRos_0
    pos:
      - 736
      - 528
    color:
      - -1
      - -1
      - -1
    z: 571
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        topic:
          type: value
          name: topic
          string: /visualization_marker_array
      persistent_params:
        []
forward_in:
  []
forward_out:
  []
connections:
  - uuid: csapex::Graph_0:|:out_2
    targets:
      - vision_plugins::GenericImageCombiner_0:|:in_0
  - uuid: csapex::Graph_0:|:out_0
    targets:
      - csapex::OutputDisplay_5:|:in_0
  - uuid: vision_plugins::GenericImageCombiner_0:|:out_0
    targets:
      - csapex::OutputDisplay_4:|:in_0
  - uuid: csapex::PointCloudToIntensityImage_0:|:out_0
    targets:
      - csapex::OutputDisplay_0:|:in_0
  - uuid: csapex::ImportRos_0:|:out_0
    targets:
      - csapex::PointCloudToIntensityImage_0:|:in_0
      - csapex::ObjectDetector_0:|:in_0
      - csapex::Graph_0:|:in_1
      - csapex::Graph_0:|:in_2
  - uuid: csapex::ObjectDetector_0:|:out_0
    targets:
      - csapex::Graph_0:|:in_0
      - csapex::CloudRenderer_0:|:in_0
  - uuid: csapex::ObjectDetector_0:|:out_1
    targets:
      - csapex::ExportRos_0:|:in_0
  - uuid: csapex::Graph_0:|:out_1
    targets:
      - csapex::OutputDisplay_1:|:in_0
      - vision_plugins::GenericImageCombiner_0:|:in_1
adapters:
  - uuid: csapex::OutputDisplay_0
    state:
      width: 819
      height: 154
  - uuid: csapex::OutputDisplay_1
    state:
      width: 2048
      height: 480
  - uuid: csapex::OutputDisplay_4
    state:
      width: 2048
      height: 480
  - uuid: csapex::OutputDisplay_5
    state:
      width: 2048
      height: 480