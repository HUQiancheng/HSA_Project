#include <ros/ros.h>
#include <tum_ics_skin_descr/Patch/TfMarkerDataPatch.h>
#include <QApplication>
#include <QDebug>
#include <QString>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>

using namespace tum_ics_skin_descr;

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "skin_force_publisher");
    ros::NodeHandle nh;
    ros::Rate rate(30); // 30 Hz publishing rate
    
    // Qt application is needed for the skin API
    QApplication app(argc, argv);
    
    // Publisher using std_msgs first (we'll switch to custom message after first build)
    ros::Publisher force_pub = nh.advertise<std_msgs::Float64MultiArray>("skin_forces", 10);
    
    // Load patch configuration
    Patch::TfMarkerDataPatch patch;
    
    // Load from your default.xml file - Note: using QString instead of std::string
    QString config_path = "/workspaces/HSA_Project/test_skin_ws/launch/configs/default.xml";
    if (!patch.load(config_path, 0)) // 0 means load first patch regardless of ID
    {
        ROS_ERROR("Failed to load patch configuration from %s", config_path.toStdString().c_str());
        return -1;
    }
    
    // Verify we have 4 cells - using numberOfCells() method
    if (patch.numberOfCells() != 4)
    {
        ROS_ERROR("Expected 4 cells, but patch has %d cells", patch.numberOfCells());
        return -1;
    }
    
    ROS_INFO("Successfully loaded patch with %d cells", patch.numberOfCells());
    
    // Let's also print the cell IDs to verify they are 1, 2, 3, 4
    QVector<int> ids = patch.cellIds();
    ROS_INFO("Cell IDs in patch:");
    for (int i = 0; i < ids.size(); i++)
    {
        ROS_INFO("  Index %d -> Cell ID %d", i, ids[i]);
    }
    
    // Create and enable data connection
    patch.createDataConnection();
    patch.enableDataConnection();
    
    // Optional: Set base frame if needed
    patch.setBaseFrame("/world");
    patch.setPose(Eigen::Affine3d::Identity());
    
    // Main loop
    while (ros::ok())
    {
        // Create message to publish
        std_msgs::Float64MultiArray msg;
        msg.data.resize(4);
        
        // Read and average forces for each cell
        for (int cellId = 1; cellId <= 4; cellId++)
        {
            try {
                // Get cell data using dataFromId - this is the correct method from the API
                Skin::Cell::Data cellData = patch.dataFromId(cellId);
                
                // Average the 3 force sensors
                double avgForce = 0.0;
                if (cellData.force.size() >= 3)
                {
                    avgForce = (cellData.force[0] + cellData.force[1] + cellData.force[2]) / 3.0;
                }
                else
                {
                    ROS_WARN_ONCE("Cell %d has %d force sensors (expected 3)", 
                                  cellId, static_cast<int>(cellData.force.size()));
                    // Average whatever sensors are available
                    for (int i = 0; i < cellData.force.size(); i++)
                    {
                        avgForce += cellData.force[i];
                    }
                    if (cellData.force.size() > 0)
                    {
                        avgForce /= cellData.force.size();
                    }
                }
                
                // Store in message (cellId 1 goes to index 0, etc.)
                msg.data[cellId - 1] = avgForce;
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("Error reading cell %d: %s", cellId, e.what());
                msg.data[cellId - 1] = 0.0;
            }
        }
        
        // Publish the message
        force_pub.publish(msg);
        
        // Optional: Print for debugging (throttled to once per second)
        ROS_INFO_THROTTLE(1.0, "Forces: [%.4f, %.4f, %.4f, %.4f]", 
                          msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
        
        // Process Qt events - important for the skin API
        QApplication::processEvents();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}