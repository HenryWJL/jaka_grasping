# Visual Grasp

In the **find_obejct_3d.launch** file, modify the following contents:
```launch
<arg name="rgb_topic"         default="/camera/color/image_raw"/>
<arg name="depth_topic"       default="/camera/depth/image_rect_raw"/>
<arg name="camera_info_topic" default="/camera/color/camera_info"/>
```
Add the following contents to the right places:
```launch
<arg name="target_frame_id"   default="/camera_link"/>
```
```launch
<param name="target_frame_id" value="$(arg target_frame_id)" type="str"/>
```
After doing so, the launch file will be like:
```launch
<launch>
	<!-- Example finding 3D poses of the objects detected -->
	<!-- $roslaunch openni_launch openni.launch depth_registration:=true -->
	
	<arg name="object_prefix"     default="object"/>
	<arg name="objects_path"      default=""/>
	<arg name="gui"               default="true"/>
	<arg name="approx_sync"       default="true"/>
	<arg name="pnp"               default="true"/>
	<arg name="tf_example"        default="true"/>
	<arg name="settings_path"     default="~/.ros/find_object_2d.ini"/>
	<arg name="target_frame_id"   default="/camera_link"/>
	
	<arg name="rgb_topic"         default="/camera/color/image_raw"/>
        <arg name="depth_topic"       default="/camera/depth/image_rect_raw"/>
        <arg name="camera_info_topic" default="/camera/color/camera_info"/>
	
	<node name="find_object_3d" pkg="find_object_2d" type="find_object_2d" output="screen">
		<param name="gui" value="$(arg gui)" type="bool"/>
		<param name="settings_path" value="$(arg settings_path)" type="str"/>
		<param name="subscribe_depth" value="true" type="bool"/>
		<param name="objects_path" value="$(arg objects_path)" type="str"/>
		<param name="object_prefix" value="$(arg object_prefix)" type="str"/>
		<param name="approx_sync" value="$(arg approx_sync)" type="bool"/>
		<param name="pnp" value="$(arg pnp)" type="bool"/>
		
		<remap from="rgb/image_rect_color" to="$(arg rgb_topic)"/>
		<remap from="depth_registered/image_raw" to="$(arg depth_topic)"/>
		<remap from="depth_registered/camera_info" to="$(arg camera_info_topic)"/>
	</node>
	
	<!-- Example of tf synchronisation with the objectsStamped message -->
	<node if="$(arg tf_example)" name="tf_example" pkg="find_object_2d" type="tf_example" output="screen">
		<param name="object_prefix" value="$(arg object_prefix)" type="str"/>
		<param name="target_frame_id" value="$(arg target_frame_id)" type="str"/>
	</node>
</launch>
```

In the **object_detection_aruco.launch** file, modify the following two arguments' values:
```launch
<launch>
     
    <arg name="markerId"        default="250"/>
    <arg name="markerSize"      default="0.4"/> 
```
