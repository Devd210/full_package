
#include <costmap_2d/costmap_helper.h>

namespace costmap_2d
{

CostmapHelper::CostmapHelper(const ros::NodeHandle &nh, const std::string &ns) : nh_(nh), namespace_(ns)
{
    access_ = std::make_shared<std::mutex>();

    footprint_sub_ = nh_.subscribe("/costmap/" + ns + "/footprint", 2, &CostmapHelper::footprintCB, this);
    size_x_sub_ = nh_.subscribe("/costmap/" + ns + "/size_x", 10, &CostmapHelper::sizeXCB, this);
    size_y_sub_ = nh_.subscribe("/costmap/" + ns + "/size_y", 10, &CostmapHelper::sizeYCB, this);
    resolution_sub_ = nh_.subscribe("/costmap/" + ns + "/resolution", 10, &CostmapHelper::resolutionCB, this);
    cost_grid_sub_ = nh_.subscribe("/costmap/" + ns + "/cost_grid", 1, &CostmapHelper::costGridCB, this);
    origin_x_sub_ = nh_.subscribe("/costmap/" + ns + "/origin_x", 1, &CostmapHelper::originXCB, this);
    origin_y_sub_ = nh_.subscribe("/costmap/" + ns + "/origin_y", 1, &CostmapHelper::originYCB, this);
    global_frame_id_sub_ = nh_.subscribe("/costmap/" + ns + "/global_frame_id", 1, &CostmapHelper::globalFrameCB, this);

    start_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/costmap" + ns + "/start_costmap");
    stop_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/costmap" + ns + "/stop_costmap");
    reset_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/costmap" + ns + "/reset_layers");
    pause_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/costmap" + ns + "/pause_costmap");
    resume_srv_client_ = nh_.serviceClient<std_srvs::Trigger>("/costmap" + ns + "/resume_costmap");
}

void CostmapHelper::start()
{
    start_srv_client_.waitForExistence();
    std_srvs::Trigger start_srv;
    if (start_srv_client_.call(start_srv))
    {
        if (start_srv.response.success)
            ROS_INFO("%s", start_srv.response.message.c_str());
        else
            ROS_WARN("Some error occured while starting the costmap!!!");
    }
}
void CostmapHelper::stop()
{
    stop_srv_client_.waitForExistence();
    std_srvs::Trigger stop_srv;
    if (start_srv_client_.call(stop_srv))
    {
        if (stop_srv.response.success)
            ROS_INFO("%s", stop_srv.response.message.c_str());
        else
            ROS_WARN("Some error occured while stopping the costmap!!!");
    }
}
void CostmapHelper::resetLayers()
{
    reset_srv_client_.waitForExistence();
    std_srvs::Trigger reset_srv;
    if (start_srv_client_.call(reset_srv))
    {
        if (reset_srv.response.success)
            ROS_INFO("%s", reset_srv.response.message.c_str());
        else
            ROS_WARN("Some error occured while starting the costmap!!!");
    }
}
void CostmapHelper::pause()
{
    pause_srv_client_.waitForExistence();
    std_srvs::Trigger pause_srv;
    if (pause_srv_client_.call(pause_srv))
    {
        if (pause_srv.response.success)
            ROS_INFO("%s", pause_srv.response.message.c_str());
        else
            ROS_WARN("Some error occured while starting the costmap!!!");
    }
}
void CostmapHelper::resume()
{
    resume_srv_client_.waitForExistence();
    std_srvs::Trigger resume_srv;
    if (start_srv_client_.call(resume_srv))
    {
        if (resume_srv.response.success)
            ROS_INFO("%s", resume_srv.response.message.c_str());
        else
            ROS_WARN("Some error occured while starting the costmap!!!");
    }
}

bool CostmapHelper::isReady()
{
    return (!costmap_global_frame_.empty() && size_x_ > 0 && size_y_ > 0 && !costmap_.empty() && resolution_ > 0 && !footprint_.empty());
}

std::string CostmapHelper::getGlobalFrameID()
{
    return costmap_global_frame_;
}

unsigned char CostmapHelper::getCost(unsigned int x, unsigned int y)
{
    std::unique_lock<std::mutex> lock(*access_);
    return (unsigned char)costmap_[y * size_x_ + x];
}

void CostmapHelper::setCost(unsigned int x, unsigned int y, unsigned char cost)
{
    //dummy function use service call here
    std::unique_lock<std::mutex> lock(*access_);
    costmap_[y * size_x_ + x] = cost;
}

bool CostmapHelper::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my)
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;

    mx = (unsigned int)((wx - origin_x_) / resolution_);
    my = (unsigned int)((wy - origin_y_) / resolution_);

    return (mx < size_x_ && my < size_y_);
}

bool CostmapHelper::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy)
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    return true;
}

std::vector<geometry_msgs::Point> CostmapHelper::getRobotFootprint()
{
    return footprint_;
}

unsigned int CostmapHelper::getSizeInCellsX()
{
    return size_x_;
}

unsigned int CostmapHelper::getSizeInCellsY()
{
    return size_y_;
}

double CostmapHelper::getOriginX()
{
    return origin_x_;
}

double CostmapHelper::getOriginY()
{
    return origin_y_;
}

double CostmapHelper::getResolution()
{
    return resolution_;
}

void CostmapHelper::originXCB(std_msgs::Float64 msg)
{
    origin_x_ = msg.data;
}
void CostmapHelper::originYCB(std_msgs::Float64 msg)
{
    origin_y_ = msg.data;
}
void CostmapHelper::resolutionCB(std_msgs::Float64 msg)
{
    resolution_ = msg.data;
}
void CostmapHelper::sizeXCB(std_msgs::UInt16 msg)
{
    size_x_ = msg.data;
}
void CostmapHelper::sizeYCB(std_msgs::UInt16 msg)
{
    size_y_ = msg.data;
}
void CostmapHelper::costGridCB(mw_msgs::CostGrid msg)
{
    std::unique_lock<std::mutex> lock(*access_);
    costmap_ = msg.cost_grid;
}
void CostmapHelper::globalFrameCB(std_msgs::String msg)
{
    costmap_global_frame_ = msg.data;
}
void CostmapHelper::footprintCB(geometry_msgs::PolygonStamped msg)
{
    footprint_.clear();
    geometry_msgs::Point p;
    for (int i = 0; i < msg.polygon.points.size(); i++)
    {
        p.x = msg.polygon.points[i].x;
        p.y = msg.polygon.points[i].y;
        p.z = msg.polygon.points[i].z; // }

        footprint_.push_back(p);
    }
}

unsigned char *CostmapHelper::getCharMap()
{
    std::unique_lock<std::mutex> lock(*access_);
    char_map_ = &costmap_[0];
    return char_map_;
}

} // namespace costmap_2d