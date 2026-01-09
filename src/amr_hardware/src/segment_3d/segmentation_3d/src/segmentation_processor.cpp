#include "segmentation_processor.h"
#include "point_processor.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

SegmentationProcessor::SegmentationProcessor() : Node("segmentation_processor_node") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    initParams();
    
    segmentation_sub_ = this->create_subscription<segmentation_msgs::msg::ObjectsSegment>(
        input_segment_topic_, 10,
        std::bind(&SegmentationProcessor::segmentationCallback, this, std::placeholders::_1));
    
    boxes3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
        output_bbx3d_topic_, 100);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/segmentation_processor/markers", 100);
    debug_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_pointcloud_topic_, 100);
    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        debug_markers_topic_, 100);
    
    // Use BEST_EFFORT QoS to match point_cloud_xyz_node publisher
    auto qos = rclcpp::QoS(10).best_effort();
    pointCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_pointcloud_ = msg;
        });
    
    RCLCPP_WARN(this->get_logger(), "SegmentationProcessor initialized");
}

void SegmentationProcessor::initParams() {
    input_segment_topic_ = "/ultralytics/segmentation/objects_segment";
    output_bbx3d_topic_ = "/segmentation_processor/bounding_boxes_3d";
    pointcloud_topic_ = "/points";
    debug_pointcloud_topic_ = "/segmentation_processor/debug_pointcloud";
    debug_markers_topic_ = "/segmentation_processor/debug_markers";
    working_frame_ = "odom";  // Default to odom (exists without SLAM), can be changed to "map" if using Nav2
    mininum_detection_threshold_ = 0.5f;
    minimum_probability_ = 0.3f;
    // Default interested classes (will be overridden by config if provided)
    // Include both tire/tyre spellings to avoid label mismatches between COCO/custom models.
    interested_classes_ = {"person", "truck", "car", "tire", "tyre"};

    this->declare_parameter("input_segment_topic", input_segment_topic_);
    this->declare_parameter("output_bbx3d_topic", output_bbx3d_topic_);
    this->declare_parameter("point_cloud_topic", pointcloud_topic_);
    this->declare_parameter("debug_pointcloud_topic", debug_pointcloud_topic_);
    this->declare_parameter("debug_markers_topic", debug_markers_topic_);
    this->declare_parameter("working_frame", working_frame_);
    this->declare_parameter("mininum_detection_threshold", mininum_detection_threshold_);
    this->declare_parameter("minimum_probability", minimum_probability_);
    this->declare_parameter("interested_classes", interested_classes_);

    this->get_parameter("input_segment_topic", input_segment_topic_);
    this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
    this->get_parameter("point_cloud_topic", pointcloud_topic_);
    this->get_parameter("debug_pointcloud_topic", debug_pointcloud_topic_);
    this->get_parameter("debug_markers_topic", debug_markers_topic_);
    this->get_parameter("working_frame", working_frame_);
    this->get_parameter("mininum_detection_threshold", mininum_detection_threshold_);
    this->get_parameter("minimum_probability", minimum_probability_);
    this->get_parameter("interested_classes", interested_classes_);
    
    // Log filtered classes
    std::string classes_str = "";
    for (size_t i = 0; i < interested_classes_.size(); ++i) {
        classes_str += interested_classes_[i];
        if (i < interested_classes_.size() - 1) classes_str += ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Class filtering enabled. Will only process: [%s]", classes_str.c_str());
    RCLCPP_WARN(this->get_logger(), "Parameters initialized");
}

void SegmentationProcessor::segmentationCallback(const segmentation_msgs::msg::ObjectsSegment::SharedPtr msg) {
    rclcpp::Time segment_stamp = msg->header.stamp;
    
    // Log received segmentation message (throttled to avoid spam)
    static size_t callback_count = 0;
    callback_count++;
    if (callback_count % 30 == 0) {  // Log every 30th call
        RCLCPP_INFO(this->get_logger(), "Received segmentation message with %zu objects", msg->objects.size());
    }
    
    // Use the latest pointcloud
    if (!latest_pointcloud_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No PointCloud2 message available for segmentation callback");
        return;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr closest_pointcloud_msg = latest_pointcloud_;

    sensor_msgs::msg::PointCloud2 local_pointcloud;
    
    // If working_frame is same as point cloud frame, skip transform
    if (working_frame_ == closest_pointcloud_msg->header.frame_id) {
        local_pointcloud = *closest_pointcloud_msg;
    } else {
        try {
            geometry_msgs::msg::TransformStamped transform;
            rclcpp::Time lookup_time = closest_pointcloud_msg->header.stamp;
            
            // Check if transform is available (with shorter timeout to reduce warnings)
            if (!tf_buffer_->canTransform(working_frame_, closest_pointcloud_msg->header.frame_id,
                                          lookup_time, rclcpp::Duration::from_seconds(0.1))) {
                // If transform not available, use point cloud frame directly
                RCLCPP_DEBUG_ONCE(this->get_logger(), 
                    "Transform from %s to %s not available, using point cloud frame directly",
                    closest_pointcloud_msg->header.frame_id.c_str(), working_frame_.c_str());
                local_pointcloud = *closest_pointcloud_msg;
                // Update working_frame to match point cloud frame for this processing
                // (but don't change the parameter)
            } else {
                transform = tf_buffer_->lookupTransform(
                    working_frame_, closest_pointcloud_msg->header.frame_id,
                    lookup_time);
                
                tf2::doTransform(*closest_pointcloud_msg, local_pointcloud, transform);
            }
        } catch (tf2::TransformException& ex) {
            // If transform fails, use point cloud frame directly
            RCLCPP_DEBUG_ONCE(this->get_logger(), "Transform error, using point cloud frame directly: %s", ex.what());
            local_pointcloud = *closest_pointcloud_msg;
        }
    }

    // Use XYZ-only point cloud to avoid requiring an RGB field in PointCloud2
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        pcl::fromROSMsg(local_pointcloud, *cloud_pcl);
    } catch (std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting PointCloud2 to PCL: %s", e.what());
        return;
    }

    if (cloud_pcl->empty()) {
        RCLCPP_ERROR(this->get_logger(), "PointCloud is empty after conversion");
        return;
    }

    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes3d_msg;
    boxes3d_msg.header.stamp = local_pointcloud.header.stamp;
    boxes3d_msg.header.frame_id = working_frame_;

    pcl::PointCloud<pcl::PointXYZ> debug_cloud;

    // Filter objects by interested_classes before processing
    int filtered_count = 0;
    for (const auto& object_segment : msg->objects) {
        // Check if this object's class is in our interested_classes list
        bool is_interested = false;
        std::string class_name_lower = object_segment.class_name;
        // Convert to lowercase for case-insensitive matching
        std::transform(class_name_lower.begin(), class_name_lower.end(), class_name_lower.begin(), ::tolower);
        
        for (const auto& interested_class : interested_classes_) {
            std::string interested_class_lower = interested_class;
            std::transform(interested_class_lower.begin(), interested_class_lower.end(), interested_class_lower.begin(), ::tolower);
            if (class_name_lower == interested_class_lower) {
                is_interested = true;
                break;
            }
        }
        
        // Skip objects not in interested_classes
        if (!is_interested) {
            filtered_count++;
            continue;
        }
        
        // Process only interested objects
        calculate_boxes(local_pointcloud, cloud_pcl, object_segment, &boxes3d_msg);

        // Add points from the segment to the debug point cloud
        for (size_t i = 0; i < object_segment.x_indices.size(); ++i) {
            int x = object_segment.x_indices[i];
            int y = object_segment.y_indices[i];

            if (x >= static_cast<int>(local_pointcloud.width) || y >= static_cast<int>(local_pointcloud.height)) {
                RCLCPP_WARN(this->get_logger(), "Point (%d, %d) is out of bounds for the point cloud", x, y);
                continue;
            }

            int pcl_index = (y * local_pointcloud.width) + x;
            const pcl::PointXYZ& point = cloud_pcl->at(pcl_index);
            if (!std::isnan(point.x)) {
                debug_cloud.push_back(point);
            }
        }
    }
    
    // Log filtering statistics (only when objects are filtered to reduce verbosity)
    if (filtered_count > 0) {
        RCLCPP_DEBUG(this->get_logger(), 
            "Filtered %d objects (not in interested_classes), processed %zu objects", 
            filtered_count, boxes3d_msg.bounding_boxes.size());
    }

    // Only publish if we have bounding boxes
    if (!boxes3d_msg.bounding_boxes.empty()) {
        boxes3d_pub_->publish(boxes3d_msg);
        publish_markers(boxes3d_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu 3D bounding boxes", boxes3d_msg.bounding_boxes.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "No 3D bounding boxes computed from %zu segmented objects", msg->objects.size());
    }
    
    publish_debug_pointcloud(debug_cloud);
    debug_markers_pub_->publish(center_markers_);
    center_markers_.markers.clear();
}

void SegmentationProcessor::publish_debug_pointcloud(const pcl::PointCloud<pcl::PointXYZ>& debug_cloud) {
    sensor_msgs::msg::PointCloud2 debug_cloud_msg;
    pcl::toROSMsg(debug_cloud, debug_cloud_msg);
    debug_cloud_msg.header.frame_id = working_frame_;
    debug_cloud_msg.header.stamp = this->now();
    debug_pointcloud_pub_->publish(debug_cloud_msg);
}

void SegmentationProcessor::calculate_boxes(const sensor_msgs::msg::PointCloud2& cloud_pc2,
                                            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_pcl,
                                            const segmentation_msgs::msg::ObjectSegment& object_segment,
                                            gb_visual_detection_3d_msgs::msg::BoundingBoxes3d* boxes) {
    // Step 1: Create a point cloud for the object mask
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    object_cloud->is_dense = false;

    // Step 2: Populate object point cloud with only valid points from the object mask
    int valid_points = 0;
    for (size_t i = 0; i < object_segment.x_indices.size(); ++i) {
        int x = object_segment.x_indices[i];
        int y = object_segment.y_indices[i];

        if (x >= static_cast<int>(cloud_pc2.width) || y >= static_cast<int>(cloud_pc2.height)) {
            RCLCPP_WARN(this->get_logger(), "Point (%d, %d) is out of bounds for the point cloud", x, y);
            continue;
        }

        int pcl_index = (y * cloud_pc2.width) + x;
        const pcl::PointXYZ& point = cloud_pcl->at(pcl_index);

        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            continue;
        }

        pcl::PointXYZ valid_point;
        valid_point.x = point.x;
        valid_point.y = point.y;
        valid_point.z = point.z;
        object_cloud->points.push_back(valid_point);
        valid_points++;
    }

    if (valid_points == 0) {
        RCLCPP_WARN(this->get_logger(), "No valid points found for object: %s", object_segment.class_name.c_str());
        return;
    }

    object_cloud->width = object_cloud->points.size();
    object_cloud->height = 1;
    RCLCPP_INFO(this->get_logger(), "Extracted %d valid points for object: %s", valid_points, object_segment.class_name.c_str());


    // Step 3-4: Segment the ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int number_of_planes = 1;  // Segment one plane (ground)
    if (!segment_cloud(object_cloud, segmented_cloud, number_of_planes)) {
        RCLCPP_WARN(this->get_logger(), "Failed to segment point cloud for object: %s", object_segment.class_name.c_str());
        return;
    }

    if (segmented_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Segmented cloud is empty for object: %s", object_segment.class_name.c_str());
        return;
    }

    // Step 5: Cluster the segmented cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    double cluster_tolerance = 0.1;  // Set clustering tolerance
    int min_cluster_size = std::min(50, static_cast<int>(segmented_cloud->points.size() / 10));  // Adaptive min cluster size
    int max_cluster_size = 40000;
    if (!cluster_cloud(segmented_cloud, best_cluster, cluster_tolerance, min_cluster_size, max_cluster_size)) {
        RCLCPP_WARN(this->get_logger(), "Failed to cluster point cloud for object: %s (segmented cloud had %zu points)", 
                   object_segment.class_name.c_str(), segmented_cloud->points.size());
        return;
    }

    if (best_cluster->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Best cluster is empty for object: %s", object_segment.class_name.c_str());
        return;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << " ****########****** Clustering processing  took " << duration.count()
              << " milliseconds." << std::endl;

    // Step 6: Compute 3D bounding box from the best cluster
    float maxx, minx, maxy, miny, maxz, minz;
    maxx = maxy = maxz = -std::numeric_limits<float>::max();
    minx = miny = minz = std::numeric_limits<float>::max();

    for (const auto& point : best_cluster->points) {
        if (std::isnan(point.x)) continue;

        maxx = std::max(point.x, maxx);
        maxy = std::max(point.y, maxy);
        maxz = std::max(point.z, maxz);
        minx = std::min(point.x, minx);
        miny = std::min(point.y, miny);
        minz = std::min(point.z, minz);
    }

    // Step 7: Create and store the bounding box message
    gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg;
    // bbx_msg.class_ = object_segment.class_name;
    bbx_msg.object_name = object_segment.class_name;
    bbx_msg.probability = object_segment.probability;
    bbx_msg.xmin = minx;
    bbx_msg.xmax = maxx;
    bbx_msg.ymin = miny;
    bbx_msg.ymax = maxy;
    bbx_msg.zmin = minz;
    bbx_msg.zmax = maxz;

    boxes->bounding_boxes.push_back(bbx_msg);
}


pcl::PointXYZ SegmentationProcessor::compute_center_point(const sensor_msgs::msg::PointCloud2& cloud_pc2,
                                                             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_pcl,
                                                             const segmentation_msgs::msg::ObjectSegment& object_segment) {
    // Timing removed - function not currently used for performance measurement

    if (object_segment.x_indices.empty() || object_segment.y_indices.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Object segment has no valid indices");
        return pcl::PointXYZ();
    }

    int center_x = std::accumulate(object_segment.x_indices.begin(), object_segment.x_indices.end(), 0) / object_segment.x_indices.size();
    int center_y = std::accumulate(object_segment.y_indices.begin(), object_segment.y_indices.end(), 0) / object_segment.y_indices.size();
    std::vector<int> xs = object_segment.x_indices;
    std::vector<int> ys = object_segment.y_indices;
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());

    int min_x = xs.front();
    int max_x = xs.back();
    int min_y = ys.front();
    int max_y = ys.back();

    int width = max_x - min_x;
    int height = max_y - min_y;

    // Convert num_samples to be in terms of height and width percentages
    int num_samples = 500; // Example number of samples
    float width_percentage = 0.12f;  // 55% of the bounding box width
    float height_percentage = 0.15f; // 35% of the bounding box height

    int width_step = static_cast<int>(width * width_percentage) / num_samples;
    int height_step = static_cast<int>(height * height_percentage) / num_samples;

    std::vector<pcl::PointXYZ> points;

    // Sampling in the region defined by the bounding box
    for (int i = -num_samples / 2; i <= num_samples / 2; ++i) {
        for (int j = -num_samples / 2; j <= num_samples / 2; ++j) {
            int new_x = center_x + i * width_step;
            int new_y = center_y + j * height_step;

            // Ensure the new points are within the image bounds
            if (new_x < 0 || new_x >= static_cast<int>(cloud_pc2.width) || new_y < 0 || new_y >= static_cast<int>(cloud_pc2.height)) {
                continue;
            }

            int pcl_index = (new_y * cloud_pc2.width) + new_x;
            pcl::PointXYZ point = cloud_pcl->at(pcl_index);

            // Check if the point is valid
            if (!std::isnan(point.x)) {
                points.push_back(point);
            }
        }
    }

    // If no valid points were found, return a default invalid point
    if (points.empty()) {
        return pcl::PointXYZ();
    }

    // Find the point with the minimum x value
    pcl::PointXYZ min_x_point = points[0];
    for (const auto& pt : points) {
        if (pt.x < min_x_point.x) {
            min_x_point = pt;
        }
    }
    return min_x_point;
}

void SegmentationProcessor::publish_markers(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d& boxes) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < boxes.bounding_boxes.size(); ++i) {
        const auto& box = boxes.bounding_boxes[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = boxes.header.frame_id;
        marker.header.stamp = boxes.header.stamp;
        marker.ns = "segmentation_3d";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = (box.xmax + box.xmin) / 2.0;
        marker.pose.position.y = (box.ymax + box.ymin) / 2.0;
        marker.pose.position.z = (box.zmax + box.zmin) / 2.0;
        marker.scale.x = (box.xmax - box.xmin);
        marker.scale.y = (box.ymax - box.ymin) ;
        marker.scale.z = box.zmax - box.zmin;

        marker.color.r = 1.0 - box.probability;
        marker.color.g = box.probability;
        marker.color.b = 0.0;
        marker.color.a = 0.6;

        marker_array.markers.push_back(marker);
    }

    markers_pub_->publish(marker_array);
}

void SegmentationProcessor::push_center_marker(const pcl::PointXYZ& center){

    static int i_ = 0;
     visualization_msgs::msg::Marker marker;
     marker.header.frame_id = working_frame_;
     marker.header.stamp = this->now();
     marker.ns = "segmentation_3d_CENTER";
     marker.id = i_++;
     marker.type = visualization_msgs::msg::Marker::CUBE;
     marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = center.z;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.3;
        marker.color.b = 0.3;
        marker.color.a = 0.6;

        center_markers_.markers.push_back(marker);

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SegmentationProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
