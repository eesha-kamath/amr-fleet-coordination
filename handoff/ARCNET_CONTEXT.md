# ARCNET CONTEXT

## REPOSITORY TREE


Folder PATH listing for volume New Volume
Volume serial number is 00000017 EA57:9F03
E:.
|   .gitignore
|   HANDOFF.md
|   
+---arcnet_api
|   |   main.py
|   |   models.py
|   |   requirements.txt
|   |   session.py
|   |   __init__.py
|   |   
|   \---__pycache__
|           main.cpython-313.pyc
|           models.cpython-313.pyc
|           session.cpython-313.pyc
|           __init__.cpython-313.pyc
|           
+---arcnet_core
|   |   allocator.py
|   |   arbiter.py
|   |   avoidance.py
|   |   baseline.py
|   |   cell.py
|   |   clock.py
|   |   config.py
|   |   conflict.py
|   |   deadlock.py
|   |   decision.py
|   |   engine.py
|   |   events.py
|   |   faults.py
|   |   fleet.py
|   |   geometry.py
|   |   grid.py
|   |   intent.py
|   |   message.py
|   |   metrics.py
|   |   motion.py
|   |   obstacle.py
|   |   peer.py
|   |   planner.py
|   |   robot.py
|   |   safety.py
|   |   scenario.py
|   |   scenarios.py
|   |   simulator.py
|   |   spatial.py
|   |   state.py
|   |   strategy.py
|   |   task.py
|   |   time.py
|   |   transport.py
|   |   validation.py
|   |   __init__.py
|   |   
|   \---__pycache__
|           allocator.cpython-313.pyc
|           arbiter.cpython-313.pyc
|           avoidance.cpython-313.pyc
|           baseline.cpython-313.pyc
|           cell.cpython-313.pyc
|           clock.cpython-313.pyc
|           config.cpython-313.pyc
|           conflict.cpython-313.pyc
|           deadlock.cpython-313.pyc
|           decision.cpython-313.pyc
|           engine.cpython-313.pyc
|           events.cpython-313.pyc
|           faults.cpython-313.pyc
|           fleet.cpython-313.pyc
|           geometry.cpython-313.pyc
|           grid.cpython-313.pyc
|           intent.cpython-313.pyc
|           message.cpython-313.pyc
|           metrics.cpython-313.pyc
|           motion.cpython-313.pyc
|           obstacle.cpython-313.pyc
|           peer.cpython-313.pyc
|           planner.cpython-313.pyc
|           robot.cpython-313.pyc
|           safety.cpython-313.pyc
|           scenario.cpython-313.pyc
|           scenarios.cpython-313.pyc
|           simulator.cpython-313.pyc
|           spatial.cpython-313.pyc
|           state.cpython-313.pyc
|           strategy.cpython-313.pyc
|           task.cpython-313.pyc
|           time.cpython-313.pyc
|           transport.cpython-313.pyc
|           validation.cpython-313.pyc
|           __init__.cpython-313.pyc
|           
+---arcnet_ros2
+---fleet_dev
|       brain_node.py
|       conflict_resolver.py
|       get_shelf_bbox.py
|       intent_codec.py
|       lamport_clock.py
|       metrics.py
|       motion_controller.py
|       path_planner.py
|       test_planner.py
|       warehouse_grid.py
|       
+---frontend
|       app.js
|       index.html
|       style.css
|       
+---handoff
|       ARCNET_APP_CODE.md
|       ARCNET_CONTEXT.md
|       ARCNET_CORE_CODE.md
|       
+---scenarios
|       blocked_aisle.json
|       four_way_choke.json
|       head_on.json
|       narrow_aisle.json
|       robot_failure.json
|       three_way_choke.json
|       t_junction.json
|       wifi_dead_zone.json
|       
+---src
|   +---aws-robomaker-small-warehouse-world
|   |       README.md
|   |       
|   \---bcr_bot
|       |   .gitignore
|       |   CHANGELOG.rst
|       |   CMakeLists.txt
|       |   Dockerfile
|       |   LICENSE
|       |   package.xml
|       |   README.md
|       |   
|       +---config
|       |       bcr_map.pgm
|       |       bcr_map.yaml
|       |       mapper_params_online_async.yaml
|       |       nav2_params.yaml
|       |       
|       +---launch
|       |       bcr_bot_gazebo_spawn.launch.py
|       |       bcr_bot_gz_spawn.launch.py
|       |       bcr_bot_ign_spawn.launch.py
|       |       bcr_bot_multi_spawn.launch.py
|       |       fleet_bringup.launch.py
|       |       gazebo.launch.py
|       |       gz.launch.py
|       |       ign.launch.py
|       |       mapping.launch.py
|       |       nav2.launch.py
|       |       rviz.launch.py
|       |       world_bringup.launch.py
|       |       
|       +---meshes
|       |   |   bcr_bot_mesh.dae
|       |   |   logo.png
|       |   |   realsense_texture.png
|       |   |   
|       |   \---kinect
|       |           d415.dae
|       |           kinect.dae
|       |           kinect.jpg
|       |           kinect.tga
|       |           
|       +---models
|       |   +---aws_robomaker_warehouse_Bucket_01
|       |   |   |   .DS_Store
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   |   .DS_Store
|       |   |   |   |   
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_Bucket_01.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_Bucket_01_collision.DAE
|       |   |           aws_robomaker_warehouse_Bucket_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_ClutteringA_01
|       |   |   |   .DS_Store
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   |   .DS_Store
|       |   |   |   |   
|       |   |   |   \---textures
|       |   |   |           .DS_Store
|       |   |   |           aws_robomaker_warehouse_ClutteringA_01.png
|       |   |   |           aws_robomaker_warehouse_ClutteringA_02.png
|       |   |   |           aws_robomaker_warehouse_ClutteringA_03.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_ClutteringA_01_collision.DAE
|       |   |           aws_robomaker_warehouse_ClutteringA_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_ClutteringC_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_ClutteringC_01.png
|       |   |   |           aws_robomaker_warehouse_ClutteringC_02.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_ClutteringC_01_collision.DAE
|       |   |           aws_robomaker_warehouse_ClutteringC_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_ClutteringD_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_ClutteringD_01.png
|       |   |   |           aws_robomaker_warehouse_ClutteringD_02.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_ClutteringD_01_collision.DAE
|       |   |           aws_robomaker_warehouse_ClutteringD_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_DeskC_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   |   .DS_Store
|       |   |   |   |   
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_DeskC_01.png
|       |   |   |           aws_robomaker_warehouse_DeskC_02.png
|       |   |   |           aws_robomaker_warehouse_DeskC_03.png
|       |   |   |           aws_robomaker_warehouse_DeskC_03.psd
|       |   |   |           aws_robomaker_warehouse_DeskC_04.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_DeskC_01_collision.DAE
|       |   |           aws_robomaker_warehouse_DeskC_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_GroundB_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_GroundB_01.png
|       |   |   |           aws_robomaker_warehouse_GroundB_02.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_GroundB_01_collision.DAE
|       |   |           aws_robomaker_warehouse_GroundB_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_Lamp_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_Lamp_01.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_Lamp_01_collision.DAE
|       |   |           aws_robomaker_warehouse_Lamp_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_PalletJackB_01
|       |   |   |   .DS_Store
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   |   .DS_Store
|       |   |   |   |   
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_PalletJackB_01.png
|       |   |   |           aws_robomaker_warehouse_PalletJackB_02.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_PalletJackB_01_collision.DAE
|       |   |           aws_robomaker_warehouse_PalletJackB_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_RoofB_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_RoofB_01.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_RoofB_01_collision.DAE
|       |   |           aws_robomaker_warehouse_RoofB_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_ShelfD_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_ShelfD_01.png
|       |   |   |           aws_robomaker_warehouse_ShelfD_02.png
|       |   |   |           aws_robomaker_warehouse_ShelfD_03.png
|       |   |   |           aws_robomaker_warehouse_ShelfD_04.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_ShelfD_01_collision.DAE
|       |   |           aws_robomaker_warehouse_ShelfD_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_ShelfE_01
|       |   |   |   .DS_Store
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   |   .DS_Store
|       |   |   |   |   
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_ShelfE_01.png
|       |   |   |           aws_robomaker_warehouse_ShelfE_02.png
|       |   |   |           aws_robomaker_warehouse_ShelfE_03.png
|       |   |   |           aws_robomaker_warehouse_ShelfE_04.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_ShelfE_01_collision.DAE
|       |   |           aws_robomaker_warehouse_ShelfE_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_ShelfF_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_ShelfF_01.png
|       |   |   |           aws_robomaker_warehouse_ShelfF_02.png
|       |   |   |           aws_robomaker_warehouse_ShelfF_03.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_ShelfF_01_collision.DAE
|       |   |           aws_robomaker_warehouse_ShelfF_01_visual.DAE
|       |   |           
|       |   +---aws_robomaker_warehouse_TrashCanC_01
|       |   |   |   model.config
|       |   |   |   model.sdf
|       |   |   |   
|       |   |   +---materials
|       |   |   |   \---textures
|       |   |   |           aws_robomaker_warehouse_TrashCanC_01.png
|       |   |   |           
|       |   |   \---meshes
|       |   |           aws_robomaker_warehouse_TrashCanC_01_collision.DAE
|       |   |           aws_robomaker_warehouse_TrashCanC_01_visual.DAE
|       |   |           
|       |   \---aws_robomaker_warehouse_WallB_01
|       |       |   model.config
|       |       |   model.sdf
|       |       |   
|       |       +---materials
|       |       |   \---textures
|       |       |           aws_robomaker_warehouse_WallB_01.png
|       |       |           
|       |       \---meshes
|       |               aws_robomaker_warehouse_WallB_01_collision.DAE
|       |               aws_robomaker_warehouse_WallB_01_visual.DAE
|       |               
|       +---res
|       |       gz.jpg
|       |       isaac.jpg
|       |       rviz.jpg
|       |       
|       +---rviz
|       |       entire_setup.rviz
|       |       map.rviz
|       |       
|       +---scripts
|       |       remapper.py
|       |       
|       +---urdf
|       |       bcr_bot.xacro
|       |       gazebo.xacro
|       |       gz.xacro
|       |       ign.xacro
|       |       macros.xacro
|       |       materials.xacro
|       |       
|       +---usd
|       |       ActionGraphFull.usd
|       |       bcr_bot.usd
|       |       scene.usd
|       |       warehouse_scene.usd
|       |       
|       \---worlds
|               empty.sdf
|               small_warehouse.sdf
|               
\---tests
    |   test_core.py
    |   
    \---__pycache__
            test_core.cpython-313.pyc
            
