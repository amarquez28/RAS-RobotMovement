#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <vector>
#include <optional>

/**
 * Structure to hold AprilTag detection data
 */
struct AprilTagData {
    int id;
    double x;          // Tag center pixel X
    double y;          // Tag center pixel Y
    double distance;   // Scalar distance to tag (metres)
    double pose_tx;    // Camera-relative X offset (metres)
    double pose_ty;    // Camera-relative Y offset (metres)
    double pose_tz;    // Camera-relative Z/depth offset (metres)
    double pose_err;   // Pose reprojection error (lower = better)
    double decision_margin; // Detection confidence (higher = better)
    int64_t timestamp_us;   // Capture timestamp in FPGA time (microseconds), -1 if unsynchronised
    bool valid;
};

/**
 * Class to read AprilTag data from NetworkTables
 * Published by the Raspberry Pi vision system
 */
class AprilTagReader {
public:
    AprilTagReader() {
        // Get the Vision table
        auto inst = nt::NetworkTableInstance::GetDefault();
        m_visionTable = inst.GetTable("Vision");
        
        // Subscribe to single tag entries
        m_tagDetectedSub = m_visionTable->GetBooleanTopic("tag_detected").Subscribe(false);
        m_tagIdSub = m_visionTable->GetIntegerTopic("tag_id").Subscribe(-1);
        m_tagXSub = m_visionTable->GetDoubleTopic("tag_x").Subscribe(0.0);
        m_tagYSub = m_visionTable->GetDoubleTopic("tag_y").Subscribe(0.0);
        m_tagDistanceSub = m_visionTable->GetDoubleTopic("tag_distance").Subscribe(-1.0);
        m_tagCountSub = m_visionTable->GetIntegerTopic("tag_count").Subscribe(0);

        // Primary tag pose (camera-relative, metres)
        m_tagPoseTxSub = m_visionTable->GetDoubleTopic("tag_pose_tx").Subscribe(0.0);
        m_tagPoseTySub = m_visionTable->GetDoubleTopic("tag_pose_ty").Subscribe(0.0);
        m_tagPoseTzSub = m_visionTable->GetDoubleTopic("tag_pose_tz").Subscribe(0.0);
        m_tagPoseErrSub = m_visionTable->GetDoubleTopic("tag_pose_err").Subscribe(-1.0);
        m_tagDecisionMarginSub = m_visionTable->GetDoubleTopic("tag_decision_margin").Subscribe(0.0);

        // Capture timestamp in FPGA time (microseconds), synced via NT4 clock offset
        m_tagTimestampSub = m_visionTable->GetIntegerTopic("tag_timestamp_us").Subscribe(-1);

        // Subscribe to array entries for multiple tags
        m_tagsIdsSub = m_visionTable->GetIntegerArrayTopic("tags_ids").Subscribe({});
        m_tagsXSub = m_visionTable->GetDoubleArrayTopic("tags_x").Subscribe({});
        m_tagsYSub = m_visionTable->GetDoubleArrayTopic("tags_y").Subscribe({});
        m_tagsDistancesSub = m_visionTable->GetDoubleArrayTopic("tags_distances").Subscribe({});
        m_tagsPoseTxSub = m_visionTable->GetDoubleArrayTopic("tags_pose_tx").Subscribe({});
        m_tagsPoseTySub = m_visionTable->GetDoubleArrayTopic("tags_pose_ty").Subscribe({});
        m_tagsPoseTzSub = m_visionTable->GetDoubleArrayTopic("tags_pose_tz").Subscribe({});

        // Heartbeat for connection monitoring
        m_heartbeatSub = m_visionTable->GetIntegerTopic("heartbeat").Subscribe(0);
        m_lastHeartbeat = 0;
    }
    
    /**
     * Check if vision system is connected and sending data
     * @return true if connected and receiving updates
     */
    bool IsConnected() {
        int64_t currentHeartbeat = m_heartbeatSub.Get();
        bool connected = (currentHeartbeat != m_lastHeartbeat);
        m_lastHeartbeat = currentHeartbeat;
        return connected;
    }
    
    /**
     * Check if any AprilTag is currently detected
     * @return true if at least one tag is detected
     */
    bool HasTarget() {
        return m_tagDetectedSub.Get();
    }
    
    /**
     * Get the number of tags currently detected
     * @return number of detected tags
     */
    int GetTagCount() {
        return m_tagCountSub.Get();
    }
    
    /**
     * Get data for the primary (first) detected tag
     * @return AprilTagData structure with tag information
     */
    AprilTagData GetPrimaryTag() {
        AprilTagData data;
        data.valid = m_tagDetectedSub.Get();
        data.id = m_tagIdSub.Get();
        data.x = m_tagXSub.Get();
        data.y = m_tagYSub.Get();
        data.distance = m_tagDistanceSub.Get();
        data.pose_tx = m_tagPoseTxSub.Get();
        data.pose_ty = m_tagPoseTySub.Get();
        data.pose_tz = m_tagPoseTzSub.Get();
        data.pose_err = m_tagPoseErrSub.Get();
        data.decision_margin = m_tagDecisionMarginSub.Get();
        data.timestamp_us = m_tagTimestampSub.Get();
        return data;
    }
    
    /**
     * Get data for all detected tags
     * @return vector of AprilTagData structures
     */
    std::vector<AprilTagData> GetAllTags() {
        std::vector<AprilTagData> tags;

        auto ids = m_tagsIdsSub.Get();
        auto xs = m_tagsXSub.Get();
        auto ys = m_tagsYSub.Get();
        auto distances = m_tagsDistancesSub.Get();
        auto txs = m_tagsPoseTxSub.Get();
        auto tys = m_tagsPoseTySub.Get();
        auto tzs = m_tagsPoseTzSub.Get();
        int64_t timestamp = m_tagTimestampSub.Get();

        // Ensure all arrays have the same size
        size_t count = std::min({ids.size(), xs.size(), ys.size(), distances.size(),
                                 txs.size(), tys.size(), tzs.size()});

        for (size_t i = 0; i < count; i++) {
            AprilTagData data;
            data.id = ids[i];
            data.x = xs[i];
            data.y = ys[i];
            data.distance = distances[i];
            data.pose_tx = txs[i];
            data.pose_ty = tys[i];
            data.pose_tz = tzs[i];
            data.pose_err = -1.0;       // Only available on primary tag
            data.decision_margin = 0.0;  // Only available on primary tag
            data.timestamp_us = timestamp;
            data.valid = true;
            tags.push_back(data);
        }

        return tags;
    }
    
    /**
     * Find a specific tag by ID
     * @param targetId The ID of the tag to find
     * @return Optional AprilTagData, nullopt if tag not found
     */
    std::optional<AprilTagData> FindTagById(int targetId) {
        auto tags = GetAllTags();
        
        for (const auto& tag : tags) {
            if (tag.id == targetId) {
                return tag;
            }
        }
        
        return std::nullopt;
    }
    
    /**
     * Get the closest detected tag
     * @return Optional AprilTagData, nullopt if no tags detected
     */
    std::optional<AprilTagData> GetClosestTag() {
        auto tags = GetAllTags();
        
        if (tags.empty()) {
            return std::nullopt;
        }
        
        AprilTagData closest = tags[0];
        for (const auto& tag : tags) {
            if (tag.distance >= 0 && tag.distance < closest.distance) {
                closest = tag;
            }
        }
        
        return closest;
    }
    
    /**
     * Update SmartDashboard with current vision data (for debugging)
     */
    void UpdateDashboard() {
        frc::SmartDashboard::PutBoolean("Vision/Connected", IsConnected());
        frc::SmartDashboard::PutBoolean("Vision/Has Target", HasTarget());
        frc::SmartDashboard::PutNumber("Vision/Tag Count", GetTagCount());
        
        auto primaryTag = GetPrimaryTag();
        if (primaryTag.valid) {
            frc::SmartDashboard::PutNumber("Vision/Primary ID", primaryTag.id);
            frc::SmartDashboard::PutNumber("Vision/Primary X", primaryTag.x);
            frc::SmartDashboard::PutNumber("Vision/Primary Y", primaryTag.y);
            frc::SmartDashboard::PutNumber("Vision/Primary Distance", primaryTag.distance);
        }
    }

private:
    std::shared_ptr<nt::NetworkTable> m_visionTable;
    
    // Single tag subscribers
    nt::BooleanSubscriber m_tagDetectedSub;
    nt::IntegerSubscriber m_tagIdSub;
    nt::DoubleSubscriber m_tagXSub;
    nt::DoubleSubscriber m_tagYSub;
    nt::DoubleSubscriber m_tagDistanceSub;
    nt::IntegerSubscriber m_tagCountSub;

    // Primary tag pose subscribers
    nt::DoubleSubscriber m_tagPoseTxSub;
    nt::DoubleSubscriber m_tagPoseTySub;
    nt::DoubleSubscriber m_tagPoseTzSub;
    nt::DoubleSubscriber m_tagPoseErrSub;
    nt::DoubleSubscriber m_tagDecisionMarginSub;

    // Capture timestamp (FPGA microseconds, synced via NT4 clock offset)
    nt::IntegerSubscriber m_tagTimestampSub;

    // Multiple tags subscribers
    nt::IntegerArraySubscriber m_tagsIdsSub;
    nt::DoubleArraySubscriber m_tagsXSub;
    nt::DoubleArraySubscriber m_tagsYSub;
    nt::DoubleArraySubscriber m_tagsDistancesSub;
    nt::DoubleArraySubscriber m_tagsPoseTxSub;
    nt::DoubleArraySubscriber m_tagsPoseTySub;
    nt::DoubleArraySubscriber m_tagsPoseTzSub;

    // Connection monitoring
    nt::IntegerSubscriber m_heartbeatSub;
    int64_t m_lastHeartbeat;
};


// ============================================================================
// USAGE EXAMPLES
// ============================================================================

/*
// Example 1: Basic usage in Robot.cpp

#include "AprilTagReader.h"

class Robot : public frc::TimedRobot {
private:
    AprilTagReader m_aprilTagReader;
    
public:
    void RobotPeriodic() override {
        // Update dashboard for debugging
        m_aprilTagReader.UpdateDashboard();
    }
    
    void TeleopPeriodic() override {
        // Check if vision system is connected
        if (!m_aprilTagReader.IsConnected()) {
            // Vision system not responding
            return;
        }
        
        // Check if we see a tag
        if (m_aprilTagReader.HasTarget()) {
            // Get primary tag data
            auto tag = m_aprilTagReader.GetPrimaryTag();
            
            // Use the data for alignment or positioning
            double x = tag.x;
            double y = tag.y;
            double distance = tag.distance;
            
            // Your robot control logic here
            // e.g., align robot to tag, drive to distance, etc.
        }
    }
};

// Example 2: Finding a specific tag

void AutonPeriodic() override {
    // Look for tag ID 5
    auto tag = m_aprilTagReader.FindTagById(5);
    
    if (tag.has_value()) {
        // Found tag 5
        double distance = tag->distance;
        double x = tag->x;
        
        // Drive to tag or perform action
    } else {
        // Tag 5 not visible, search for it
    }
}

// Example 3: Working with multiple tags

void ProcessMultipleTags() {
    auto tags = m_aprilTagReader.GetAllTags();
    
    for (const auto& tag : tags) {
        // Process each visible tag
        if (tag.id == 1) {
            // This is the scoring position
        } else if (tag.id == 2) {
            // This is the loading zone
        }
    }
}

// Example 4: Get closest tag

void DriveToClosestTag() {
    auto closestTag = m_aprilTagReader.GetClosestTag();
    
    if (closestTag.has_value()) {
        double distance = closestTag->distance;
        
        if (distance > 1.0) {
            // Drive forward
        } else {
            // We're close enough
        }
    }
}
*/